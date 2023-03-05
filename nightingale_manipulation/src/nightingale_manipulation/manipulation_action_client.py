#!/usr/bin/env python3

import actionlib
import rospy
import math
import time

# joint
from moveit_action_handlers.msg import (
    PropertyValuePair,
    MoveToJointsMoveItAction,
    MoveToJointsMoveItGoal,
)

# cartesian
from moveit_action_handlers.msg import (
    MoveToPoseMoveItAction,
    MoveToPoseMoveItGoal,
    PoseStamped,
)
from geometry_msgs.msg import Pose

from nightingale_msgs.srv import (
    RobotConfigurationLookup,
    RobotConfigurationLookupRequest,
)

# Kinematics
from sensor_msgs.msg import JointState
import tf_conversions, tf2_ros
from geometry_msgs.msg import Pose as GeometryPose
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion

from nightingale_manipulation.planning_scene_interface import PlanningSceneInterface
from nightingale_manipulation.trajectory_intercept_server import TrajectoryInterceptServer

MoveItActionHandlerSuccess = "Success"


# overriding the geometry msgs Pose class to add tolerance on equivalence check
class Pose(GeometryPose):
    def __init__(self, tolerance=0.01):
        super().__init__()
        self.tol = tolerance

    def __eq__(self, other):
        if type(other) not in [GeometryPose, Pose]:
            return False
        return all(
            [
                math.isclose(self.position.x, other.position.x, rel_tol=self.tol),
                math.isclose(self.position.y, other.position.y, rel_tol=self.tol),
                math.isclose(self.position.z, other.position.z, rel_tol=self.tol),
                math.isclose(self.orientation.x, other.orientation.x, rel_tol=self.tol),
                math.isclose(self.orientation.y, other.orientation.y, rel_tol=self.tol),
                math.isclose(self.orientation.z, other.orientation.z, rel_tol=self.tol),
                math.isclose(self.orientation.w, other.orientation.w, rel_tol=self.tol),
            ]
        )

    def copy(self):
        new_pose = Pose()
        new_pose.position.x = self.position.x
        new_pose.position.y = self.position.y
        new_pose.position.z = self.position.z
        new_pose.orientation.x = self.orientation.x
        new_pose.orientation.y = self.orientation.y
        new_pose.orientation.z = self.orientation.z
        new_pose.orientation.w = self.orientation.w
        return new_pose


class Orientation:
    def __init__(self, roll=None, pitch=None, yaw=None, tolerance=0.01):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.tol = tolerance

    def __eq__(self, other):
        if not type(other) is Orientation:
            return False
        return all(
            [
                math.isclose(self.roll, other.roll, rel_tol=self.tol),
                math.isclose(self.pitch, other.pitch, rel_tol=self.tol),
                math.isclose(self.yaw, other.yaw, rel_tol=self.tol),
            ]
        )


def joint_goal(
    joint_values: list, joint_names: list, eev=0.5, eea=0.5, timeout=5
) -> MoveToJointsMoveItGoal:
    goal = MoveToJointsMoveItGoal()
    goal.endEffectorVelocity = eev
    goal.endEffectorAcceleration = eea
    goal.timeoutSeconds = timeout
    for i in range(len(joint_names)):
        joint_pair = PropertyValuePair()
        joint_pair.name = joint_names[i]
        joint_pair.value = joint_values[i]
        goal.joint_pairs.append(joint_pair)
    return goal


def cartesian_goal(
    point: Point,
    ref_link,
    orientation: Orientation,
    eev=0.5,
    eea=0.5,
    mode=0,
    timeout=20,
) -> MoveToPoseMoveItGoal:
    goal = MoveToPoseMoveItGoal()
    goal.constraint_mode = mode
    goal.endEffectorVelocity = eev
    goal.endEffectorAcceleration = eea
    goal.timeoutSeconds = timeout
    target_pose = PoseStamped()
    target_pose.header.frame_id = ref_link
    target_pose.pose.position = point
    target_pose.pose.orientation.roll = orientation.roll
    target_pose.pose.orientation.pitch = orientation.pitch
    target_pose.pose.orientation.yaw = orientation.yaw
    goal.target_pose = target_pose
    return goal


class ManipulationJointControl:
    def __init__(
        self,
        left_joint_names,
        right_joint_names,
        left_home_values,
        right_home_values,
        joint_tolerance=0.01,
    ):
        self.left_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/left_arm/joint_ctrl", MoveToJointsMoveItAction
        )
        self.left_arm.wait_for_server()

        self.right_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/right_arm/joint_ctrl", MoveToJointsMoveItAction
        )
        self.right_arm.wait_for_server()

        self.left_arm_joint_names = left_joint_names
        self.right_arm_joint_names = right_joint_names
        self.left_arm_home_joint_values = left_home_values
        self.right_arm_home_joint_values = right_home_values

        self._right_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self._left_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self._joint_tolerance = joint_tolerance

        rospy.loginfo("Manipulation Control: Initialized joint ctrl")

    def update_joint_states(self):
        """
        Updates the _right_joint_states and _left_joint_states members
        @return: True if successful
        """
        try:
            joint_states = rospy.client.wait_for_message(
                "/joint_states", JointState, timeout=0.2
            )
            pos = joint_states.position
            names = joint_states.name

        except rospy.ROSException:
            rospy.logwarn(
                "Unable to get data from /joint_states, wait_for_message timeout"
            )
            return False

        names_pos_dict = {}
        for i in range(len(names)):
            names_pos_dict[names[i]] = pos[i]

        self._right_joint_states = []
        self._left_joint_states = []

        # probably a redundant check but making sure the joint names exist from the /joint_states data
        if not (
            all(key in names_pos_dict.keys() for key in self.right_arm_joint_names)
            and all(key in names_pos_dict.keys() for key in self.left_arm_joint_names)
        ):
            rospy.logerr(
                "Invalid or missing joint name in data received from /joint_states"
            )
            return False

        for i in range(7):
            self._right_joint_states.append(
                names_pos_dict[self.right_arm_joint_names[i]]
            )
            self._left_joint_states.append(names_pos_dict[self.left_arm_joint_names[i]])

        return True

    def get_joint_states(self):
        if self.update_joint_states():
            return {
                "left_arm": self._left_joint_states.copy(),
                "right_arm": self._right_joint_states.copy(),
            }
        return False

    def verify_joint_target(self, joint_target: list, joint_states: list) -> bool:
        """
        Verifies that the joint target list is valid, and that it is not the same as the current joint state
        @param joint_target:
        @param joint_states:
        @return: True if any targets are different from the current state,
                 False if all targets are the same as current state
        """

        if len(joint_target) != 7:
            raise Exception("Invalid joint value length")
        error_count = 7
        for i in range(7):
            if math.isclose(
                abs(joint_target[i]),
                abs(joint_states[i]),
                rel_tol=self._joint_tolerance,
            ):
                error_count -= 1
        return error_count > 0

    def cmd_right_arm(self, joint_target: list) -> bool:
        """
        Commands movement of the right arm through the right_arm action server
        @param joint_target: list of target joint values
        @return: result of verify_joint_target and action server state if blocking
        """
        if not self.update_joint_states():
            rospy.logerr("update joint states failed")
            return False
        if not self.verify_joint_target(joint_target, self._right_joint_states):
            return True

        goal = joint_goal(joint_target, self.right_arm_joint_names)
        self.right_arm.send_goal(goal)

        self.right_arm.wait_for_result()
        status = self.right_arm.get_result().status
        if status == MoveItActionHandlerSuccess:
            return True
        rospy.logwarn("cmd_right_arm failed with status: " + str(status))
        return False

    def cmd_left_arm(self, joint_target):
        """
        Commands the movement of the left arm through the left_arm action server
        @param joint_target: list of target joint values
        @return: boolean result of verify_joint_target and action server state if blocking
        """
        if not self.update_joint_states():
            rospy.logerr("update joint states failed")
            return False
        if not self.verify_joint_target(joint_target, self._left_joint_states):
            return True

        goal = joint_goal(joint_target, self.left_arm_joint_names)
        self.left_arm.send_goal(goal)

        self.left_arm.wait_for_result()
        status = self.left_arm.get_result().status
        if status == MoveItActionHandlerSuccess:
            return True
        rospy.logwarn("cmd_left_arm failed with status: " + str(status))
        return False

    def home(self):
        left_status = self.cmd_left_arm(self.left_arm_home_joint_values)
        right_status = self.cmd_right_arm(self.right_arm_home_joint_values)
        return left_status and right_status


class ManipulationGripperControl:
    def __init__(
        self, left_gripper_names, right_gripper_names, opened_values, closed_values
    ):
        self.left_gripper = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/left_arm/gripper_ctrl", MoveToJointsMoveItAction
        )
        self.left_gripper.wait_for_server()

        self.right_gripper = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/right_arm/gripper_ctrl", MoveToJointsMoveItAction
        )
        self.right_gripper.wait_for_server()

        # TODO: replace with service call
        self.left_gripper_joint_names = left_gripper_names
        self.right_gripper_joint_names = right_gripper_names
        self.open_joint_values = opened_values
        self.closed_joint_values = closed_values

        self.gripper_lock = {"left": True, "right": True}
        self.unlock_left_gripper()
        self.unlock_right_gripper()

        rospy.loginfo("Manipulation Control: Initialized gripper ctrl")

    def cmd_right_gripper(self, joint_target):
        goal = joint_goal(joint_target, self.right_gripper_joint_names)
        self.right_gripper.send_goal(goal)
        self.right_gripper.wait_for_result()
        return self.right_gripper.get_result().status == MoveItActionHandlerSuccess

    def cmd_left_gripper(self, joint_target):
        goal = joint_goal(joint_target, self.left_gripper_joint_names)
        self.left_gripper.send_goal(goal)
        self.left_gripper.wait_for_result()
        return self.left_gripper.get_result().status == MoveItActionHandlerSuccess

    def lock_right_gripper(self):
        self.gripper_lock["right"] = False

    def unlock_right_gripper(self):
        self.gripper_lock["right"] = True

    def lock_left_gripper(self):
        self.gripper_lock["left"] = False

    def unlock_left_gripper(self):
        self.gripper_lock["left"] = True

    def open_left(self):
        return self.cmd_left_gripper(self.open_joint_values)

    def close_left(self):
        return self.cmd_left_gripper(self.closed_joint_values)

    def open_right(self):
        return self.cmd_right_gripper(self.open_joint_values)

    def close_right(self):
        return self.cmd_right_gripper(self.closed_joint_values)


class ManipulationCartesianControl:
    def __init__(self, prefix):
        self.arm = actionlib.simple_action_client.SimpleActionClient(
            f"/moveit_action_handlers/{prefix}_arm/cartesian_ctrl",
            MoveToPoseMoveItAction,
        )
        self.arm.wait_for_server()
        self.prefix = prefix
        self.ee_ctrl_mode = 0
        self.orientation = Orientation()
        self.pose = Pose()
        self.ref_link = "base_link"
        self.ee_link = f"{prefix}_ee_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(1)
        if self.update_ee_pose():
            rospy.loginfo(
                f"Manipulation Control: {prefix} arm cartesian control initialized"
            )
        else:
            rospy.logerr(f"Manipulation Control: {prefix} arm failed to initialize")

    def update_ee_pose(self):
        """
        Uses the robot transform to update the end effector Pose and Orientation
        @return: True if successful
        """
        try:
            link_transform = self.tf_buffer.lookup_transform(
                self.ref_link, self.ee_link, rospy.Time(0)
            )
        except Exception as e:
            rospy.logerr(str(e))
            rospy.logerr(f"{self.prefix} arm transformation lookup failed")
            return False

        q = link_transform.transform.rotation

        quaternion_list = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        self.orientation.roll = roll
        self.orientation.pitch = pitch
        self.orientation.yaw = yaw

        p = link_transform.transform.translation
        self.pose.position = p
        self.pose.orientation = q
        return True

    def cmd_arm(self, pose: GeometryPose, ref_link="upper_body_link", ee_relative=False):
        """
        Commands the arm to move to a target Pose in the cartesian space
        @param pose: type geometry_msgs.msg Pose, also accepts manipulation_action_client.Pose type
        @param ref_link: the link to move with respect to
        @param ee_relative: if true, use ee_link instead of base_link for reference link
        @return: Result of action
        """
        if not self.update_ee_pose():
            return False
        if self.pose == pose:
            return True

        if ee_relative:
            ref_link = self.ee_link
        # locked end effector control mode
        if self.ee_ctrl_mode:
            goal = cartesian_goal(
                pose.position,
                ref_link,
                self.orientation,
                mode=self.ee_ctrl_mode,
            )

        # free end effector control mode
        else:
            quaternion_list = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
            euler_orientation = Orientation(roll=roll, pitch=pitch, yaw=yaw)
            goal = cartesian_goal(pose.position, ref_link, euler_orientation)

        self.arm.send_goal(goal)
        self.arm.wait_for_result()
        return self.arm.get_result().status == MoveItActionHandlerSuccess

    def cmd_position(self, point: Point, ref_link="upper_body_link", ee_fixed=True):
        if type(point) is not Point:
            return False
        ee_prev = self.ee_ctrl_mode
        if ee_fixed:
            self.set_fixed_ee_ctrl_mode()
        else:
            self.set_free_ee_ctrl_mode()

        pose = self.get_pose()
        goal = GeometryPose()
        goal.position = point
        goal.orientation = pose.orientation
        status = self.cmd_arm(goal, ref_link=ref_link)

        self.ee_ctrl_mode = ee_prev
        return status

    def cmd_orientation(self, q: Quaternion):
        if type(q) is not Quaternion:
            return False
        pose = self.get_pose()
        goal = GeometryPose()
        goal.orientation = q
        goal.position = pose.position
        return self.cmd_arm(goal, ref_link="base_link")

    def set_fixed_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 1

    def set_free_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 0

    def set_ref_link(self, ref):
        self.ref_link = ref

    def get_pose(self):
        if not self.update_ee_pose():
            return False
        return self.pose.copy()

    def get_euler_orientation(self):
        if not self.update_ee_pose():
            return False
        return self.orientation


class ManipulationControl:
    def __init__(self):
        rospy.loginfo("Manipulation Control: Starting manipulation initialization...")

        robot_config_service = "/nightingale/robot_configuration_lookup"
        rospy.loginfo(f"Manipulation Control: waiting for robot configuration service")
        rospy.wait_for_service(robot_config_service)
        rospy.loginfo(f"Manipulation Control: found robot configuration service")
        lookup_client = rospy.ServiceProxy(
            robot_config_service, RobotConfigurationLookup
        )
        try:
            response = lookup_client("home", RobotConfigurationLookupRequest.LEFT_ARM)
            self.left_arm_joint_names = response.jnt_state.name
            self.left_arm_home_joint_values = response.jnt_state.position

            response = lookup_client("home", RobotConfigurationLookupRequest.RIGHT_ARM)
            self.right_arm_joint_names = response.jnt_state.name
            self.right_arm_home_joint_values = response.jnt_state.position

            response = lookup_client(
                "handoff", RobotConfigurationLookupRequest.RIGHT_ARM
            )
            self.right_arm_handoff_joint_values = response.jnt_state.position

            response = lookup_client(
                "open_gripper", RobotConfigurationLookupRequest.LEFT_GRIPPER
            )
            self.left_gripper_joint_names = response.jnt_state.name
            self.left_gripper_open_joint_values = response.jnt_state.position

            response = lookup_client(
                "closed_gripper", RobotConfigurationLookupRequest.LEFT_GRIPPER
            )
            self.left_gripper_closed_joint_values = response.jnt_state.position

            response = lookup_client(
                "open_gripper", RobotConfigurationLookupRequest.RIGHT_GRIPPER
            )
            self.right_gripper_joint_names = response.jnt_state.name
            self.right_gripper_open_joint_values = response.jnt_state.position

            response = lookup_client(
                "closed_gripper", RobotConfigurationLookupRequest.RIGHT_GRIPPER
            )
            self.right_gripper_closed_joint_values = response.jnt_state.position

        except rospy.ServiceException as e:
            rospy.logerr(
                f"Nightingale Manipulation Control failed to call robot configuration lookup service {e}",
            )
            # this should be a hard failure because joint control initialization depends on the service working
            raise Exception("Manipulation Control: Initialization failed")

        self.tries = 3
        self.jnt_ctrl = ManipulationJointControl(
            self.left_arm_joint_names,
            self.right_arm_joint_names,
            self.left_arm_home_joint_values,
            self.right_arm_home_joint_values,
        )

        self.gpr_ctrl = ManipulationGripperControl(
            self.left_gripper_joint_names,
            self.right_gripper_joint_names,
            self.left_gripper_open_joint_values,
            self.left_gripper_closed_joint_values,
        )
        self.right_cartesian = ManipulationCartesianControl("right")
        self.left_cartesian = ManipulationCartesianControl("left")
        self.planning_scene = PlanningSceneInterface()
        self.trajectory_inversion_server = TrajectoryInterceptServer(
            "/movo/right_arm_controller/follow_joint_trajectory"
        )

    def home_right(self, tries=3):
        # CAUTION: This function should only ever home the arms. Don't add homing of other things here
        # right gripper is openend on bootup by kinova. not sure where, but not in init
        def home_right_internal():
            #if not self.gpr_ctrl.close_right():
            #    rospy.logerr("ManipulationControl failed to close right gripper")
            #    return False

            return self.jnt_ctrl.cmd_right_arm(
                self.jnt_ctrl.right_arm_home_joint_values
            )

        for _ in range(tries):
            if home_right_internal():
                return True
        return False

    def retract_right(self, tries=3):
        def reverse_previous_path():
            return self.trajectory_inversion_server.send_reverse_trajectory()

        def home_right_internal():
            # self.planning_scene.add_box()
            home_pose = GeometryPose()
            # TODO get this from the service
            home_pose.position.x = 0.356
            home_pose.position.y = 0.003
            home_pose.position.z = 0.051
            home_pose.orientation.x = -0.510
            home_pose.orientation.y = -0.493
            home_pose.orientation.z = 0.464
            home_pose.orientation.w = 0.530
            if not self.right_cartesian.cmd_position(home_pose.position):
                rospy.logerr("ManipulationControl failed to move right arm")
                # self.planning_scene.remove_box()
                return False
            # self.planning_scene.remove_box()
            return True

        for _ in range(tries):
            if reverse_previous_path():
                return True
        return False

    def home_left(self, tries=3):
        def home_left_internal():
            # if not self.gpr_ctrl.close_left():
            #    rospy.logerr("ManipulationControl failed to close left gripper")
            #    return False
            if not self.jnt_ctrl.cmd_left_arm(self.jnt_ctrl.left_arm_home_joint_values):
                rospy.logerr("ManipulationControl failed to home left arm")
                return False
            return True

        for _ in range(tries):
            if home_left_internal():
                return True
        return False

    def extend_handoff(self, goal_point: Point, tries=3):
        def extend_handoff_internal():
            return self.right_cartesian.cmd_position(goal_point, ref_link="base_link")

        for _ in range(tries):
            if extend_handoff_internal():
                return True
        return False

    def extend_restock(self, tries=3):
        # TODO: these should come from some param server
        def extend_restock_internal():
            restock_pose = GeometryPose()
            restock_pose.position.x = 0.807
            restock_pose.position.y = 0.053
            restock_pose.position.z = 0.278
            status = self.right_cartesian.cmd_position(restock_pose.position)
            return status

        for _ in range(tries):
            if extend_restock_internal():
                return True
        return False

    def open_left_gripper(self):
        return self.gpr_ctrl.open_left()

    def open_right_gripper(self):
        return self.gpr_ctrl.open_right()

    def close_left_gripper(self):
        return self.gpr_ctrl.close_left()

    def close_right_gripper(self):
        return self.gpr_ctrl.close_right()


# test code
if __name__ == "__main__":
    rospy.init_node("manipulation_control")
    manipulation = ManipulationControl()

    rospy.loginfo("going home")
    assert manipulation.home_left()
    assert manipulation.jnt_ctrl.cmd_right_arm(manipulation.jnt_ctrl.right_arm_home_joint_values)

    for _ in range(20):
        rospy.loginfo("restocking")
        assert manipulation.extend_restock()
        rospy.loginfo("retracting")
        assert manipulation.retract_right()
        rospy.loginfo("extending handoff")
        assert manipulation.extend_handoff(Point(0.86, -0.13, 0.9))
        rospy.loginfo("retracting")
        assert manipulation.retract_right()

    rospy.loginfo("going home")
    assert manipulation.home_left()
    assert manipulation.home_right()
