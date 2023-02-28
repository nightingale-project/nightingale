#!/usr/bin/env python3

import actionlib
import rospy
import math
import time

# Joint
from moveit_action_handlers.msg import PropertyValuePair
from moveit_action_handlers.msg import MoveToJointsMoveItAction
from moveit_action_handlers.msg import MoveToJointsMoveItGoal

# Cartesian
from moveit_action_handlers.msg import MoveToPoseMoveItAction
from moveit_action_handlers.msg import MoveToPoseMoveItGoal
from moveit_action_handlers.msg import PoseStamped

# Kinematics
from sensor_msgs.msg import JointState
import tf_conversions, tf2_ros
from geometry_msgs.msg import Pose as GeometryPose
from geometry_msgs.msg import Quaternion, Point, Vector3
from tf.transformations import euler_from_quaternion

# TODO: replace with service call, requires refactoring of service to include gripper info
CFG = rospy.get_param("/nightingale_utils/joint_configurations")


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
    timeout=60,
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
    def __init__(self, joint_tolerance=0.01):
        self.left_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/left_arm/joint_ctrl", MoveToJointsMoveItAction
        )
        self.left_arm.wait_for_server()

        self.right_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/right_arm/joint_ctrl", MoveToJointsMoveItAction
        )
        self.right_arm.wait_for_server()

        self.head = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/head/joint_ctrl", MoveToJointsMoveItAction
        )
        rospy.loginfo("Waiting for pan tilt joint moveit action server")
        self.head.wait_for_server()

        self.torso = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/torso/joint_ctrl", MoveToJointsMoveItAction
        )
        rospy.loginfo("Waiting for torso joint moveit action server")
        self.torso.wait_for_server()

        # TODO: replace with service call
        self.left_arm_joint_names = CFG["left_arm_home"]["names"]
        self.right_arm_joint_names = CFG["right_arm_home"]["names"]
        self.left_arm_home_joint_values = CFG["left_arm_home"]["joints"]
        self.right_arm_home_joint_values = CFG["right_arm_home"]["joints"]

        self.torso_joint_names = CFG["home"]["torso"]["names"]
        self.head_joint_names = CFG["home"]["head"]["names"]
        self.torso_home_joint_values = CFG["home"]["torso"]["joints"]
        self.head_home_joint_values = CFG["home"]["head"]["joints"]

        self._right_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self._left_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self._torso_joint_states = [0]
        self._head_joint_states = [0, 0]
        self._joint_tolerance = joint_tolerance

        self._left_joint_target = None
        self._right_joint_target = None
        self._head_joint_target = None
        self._torso_joint_target = None

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
        self._torso_joint_states = []
        self._head_joint_states = []

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

        self._head_joint_states.append(names_pos_dict[self.head_joint_names[0]])
        self._head_joint_states.append(names_pos_dict[self.head_joint_names[1]])

        self._torso_joint_states.append(names_pos_dict[self.torso_joint_names[0]])

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

    def cmd_torso(self, joint_target: list) -> bool:
        """
        Commands the movement of the torso through the joint_space action server
        @param joint_target: list of target joint values
        @param execute: if True, arm will move without waiting, else self.execute() must be called after
        @return: status
        """
        if not self.update_joint_states():
            return False

        try:
            if not len(joint_target) == 1:
                return False
        except TypeError:
            return False

        goal = joint_goal(joint_target, self.torso_joint_names)
        self.torso.send_goal(goal)
        return self.torso.wait_for_result()

    def cmd_head(self, joint_target: list) -> bool:
        """
        Commands the movement of the pan tilt through the joint_space action server
        @param joint_target: list of target joint values
        @param execute: if True, arm will move without waiting, else self.execute() must be called after
        @return: status
        """
        if not self.update_joint_states():
            return False

        try:
            if not len(joint_target) == 2:
                return False
        except TypeError:
            return False

        goal = joint_goal(joint_target, self.head_joint_names)
        self.head.send_goal(goal)
        return self.head.wait_for_result()

    def cmd_right_arm(self, joint_target: list) -> bool:
        """
        Commands movement of the right arm through the right_arm action server
        @param joint_target: list of target joint values
        @param blocking: default True, if blocking, function will wait for action server response
        @return: result of verify_joint_target and action server state if blocking
        """
        if not self.update_joint_states():
            return False
        if not self.verify_joint_target(joint_target, self._right_joint_states):
            return True

        goal = joint_goal(joint_target, self.right_arm_joint_names)
        self.right_arm.send_goal(goal)
        return self.right_arm.wait_for_result()

    def cmd_left_arm(self, joint_target: list) -> bool:
        """
        Commands the movement of the left arm through the left_arm action server
        @param joint_target: list of target joint values
        @param blocking: default True, if blocking, function will wait for action server response
        @return: boolean result of verify_joint_target and action server state if blocking
        """
        if not self.update_joint_states():
            return False
        if not self.verify_joint_target(joint_target, self._left_joint_states):
            return True

        goal = joint_goal(joint_target, self.left_arm_joint_names)
        self.left_arm.send_goal(goal)
        return self.left_arm.wait_for_result()

    def home(self):
        status = self.cmd_left_arm(self.left_arm_home_joint_values)
        status &= self.cmd_right_arm(self.right_arm_home_joint_values)

        status &= self.cmd_head(self.head_home_joint_values)
        status &= self.cmd_torso(self.torso_home_joint_values)

        return status


class ManipulationGripperControl:
    def __init__(self):
        self.left_gripper = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/left_arm/gripper_ctrl", MoveToJointsMoveItAction
        )
        self.left_gripper.wait_for_server()

        self.right_gripper = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/right_arm/gripper_ctrl", MoveToJointsMoveItAction
        )
        self.right_gripper.wait_for_server()

        # TODO: replace with service call
        opened = CFG["open_gripper"]
        closed = CFG["closed_gripper"]
        self.right_open_goal = joint_goal(
            [opened, opened, opened], CFG["right_gripper"]["joints"]
        )
        self.right_closed_goal = joint_goal(
            [closed, closed, closed], CFG["right_gripper"]["joints"]
        )

        self.left_open_goal = joint_goal(
            [opened, opened, opened], CFG["left_gripper"]["joints"]
        )
        self.left_closed_goal = joint_goal(
            [closed, closed, closed], CFG["left_gripper"]["joints"]
        )

        self.gripper_lock = {"left": True, "right": True}
        self.unlock_left_gripper()
        self.unlock_right_gripper()

    def cmd_right_gripper(self, goal, blocking=True):
        if not self.gripper_lock["right"]:
            return False

        self.right_gripper.send_goal(goal)
        self.right_gripper.wait_for_result()
        return self.right_gripper.get_result().status == "Success"

    def cmd_left_gripper(self, goal):
        if not self.gripper_lock["left"]:
            return False

        self.left_gripper.send_goal(goal)
        self.left_gripper.wait_for_result()
        return self.left_gripper.get_result().status == "Success"

    def lock_right_gripper(self):
        self.gripper_lock["right"] = False

    def unlock_right_gripper(self):
        self.gripper_lock["right"] = True

    def lock_left_gripper(self):
        self.gripper_lock["left"] = False

    def unlock_left_gripper(self):
        self.gripper_lock["left"] = True

    def open_left(self):
        return self.cmd_left_gripper(self.left_open_goal)

    def close_left(self):
        return self.cmd_left_gripper(self.left_closed_goal)

    def open_right(self):
        return self.cmd_right_gripper(self.right_open_goal)

    def close_right(self):
        return self.cmd_right_gripper(self.right_closed_goal)


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
        # TODO: include correct home pose
        self.home_pose = None

        # self.robot = Robot.from_parameter_server()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        time.sleep(1)
        if self.update_ee_pose():
            rospy.loginfo(f"{prefix} arm cartesian control initialized")
        else:
            rospy.logerr(f"{prefix} arm failed to initialize")

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

    def cmd_arm(self, pose: GeometryPose, blocking=True, ee_relative=False):
        """
        Commands the arm to move to a target Pose in the cartesian space
        @param pose: type geometry_msgs.msg Pose, also accepts manipulation_action_client.Pose type
        @param blocking: if true, wait for action server result
        @param ee_relative: if true, use ee_link instead of base_link for reference link
        @return: Result of action
        """
        if not self.update_ee_pose():
            return False
        if self.pose == pose:
            return True

        if ee_relative:
            ref_link = self.ee_link
        else:
            ref_link = self.ref_link

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
            (roll, pitch, yaw) = euler_from_quaternion(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
            euler_orientation = Orientation(roll=roll, pitch=pitch, yaw=yaw)
            goal = cartesian_goal(pose.position, ref_link, euler_orientation)

        rospy.loginfo("goal: ")
        rospy.loginfo(goal)

        self.arm.send_goal(goal)
        if blocking:
            self.arm.wait_for_result()
            return self.arm.get_result().status == "Success"
        return True

    def cmd_position(self, point: Point, ee_fixed=True):
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
        status = self.cmd_arm(goal)

        self.ee_ctrl_mode = ee_prev
        return status

    def cmd_orientation(self, q: Quaternion):
        if type(q) is not Quaternion:
            return False
        pose = self.get_pose()
        goal = GeometryPose()
        goal.orientation = q
        goal.position = pose.position
        return self.cmd_arm(goal)

    def set_fixed_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 1

    def set_free_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 0

    def set_ref_link(self, ref):
        self.ref_link = ref

    def approximate_home(self):
        self.set_ref_link("base_link")
        self.set_free_ee_ctrl_mode()

        home_point = Point()
        home_point.x = 0.427
        home_point.y = -0.03
        home_point.z = 0.616

        home_quat = Quaternion()
        home_quat.x = -0.537
        home_quat.y = -0.595
        home_quat.z = 0.395
        home_quat.w = 0.447

        aprox_home_point = Point()
        aprox_home_point.x = 0.727
        aprox_home_point.y = -0.03
        aprox_home_point.z = 0.616

        s1 = self.cmd_position(aprox_home_point)
        s2 = self.cmd_orientation(home_quat)
        s3 = self.cmd_position(home_point)
        return all([s1, s2, s3])

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
        rospy.loginfo("starting manipulation initialization, waiting for servers...")
        self.jnt_ctrl = ManipulationJointControl()
        rospy.loginfo("initialized joint ctrl")
        self.gpr_ctrl = ManipulationGripperControl()
        rospy.loginfo("initialized gripper ctrl")

        self.right_cartesian = ManipulationCartesianControl("right")
        self.left_cartesian = ManipulationCartesianControl("left")

        self.right_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self.left_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self.linear_joint_state = [0]
        self.right_ee_pose = None
        self.left_ee_pose = None
        rospy.loginfo("Manipulation initialization successful")

    def home(self):
        # CAUTION: This function should only ever home the arms. Don't add homing of other things here
        # right gripper is openend on bootup by kinova. not sure where, but not in init
        if not self.gpr_ctrl.close_right():
            return False
        if not self.jnt_ctrl.cmd_left_arm(self.jnt_ctrl.left_arm_home_joint_values):
            return False
        home_pose = GeometryPose()
        # TODO get this from the service
        home_pose.position.x = 0.581
        home_pose.position.y = 0.003
        home_pose.position.z = 0.637
        home_pose.orientation.x = -0.456
        home_pose.orientation.y = -0.583
        home_pose.orientation.z = 0.430
        home_pose.orientation.w = 0.517
        if not self.right_cartesian.cmd_orientation(home_pose.orientation):
            return False
        if not self.right_cartesian.cmd_position(home_pose.position, True):
            return False
        return True

    def extend_handoff(self, goal_point: Point):
        return self.right_cartesian.cmd_position(goal_point, True)

    def extend_restock(self):
        # TODO: these should come from some param server
        restock_pose = GeometryPose()
        restock_pose.position.x = 0.807
        restock_pose.position.y = 0.053
        restock_pose.position.z = 0.978
        return self.right_cartesian.cmd_position(restock_pose.position, True)


# test code
if __name__ == "__main__":
    rospy.init_node("manipulation_control")
    manipulation = ManipulationControl()
    manipulation.gpr_ctrl.open_right()

    manipulation.jnt_ctrl.cmd_torso(manipulation.jnt_ctrl.torso_home_joint_values)
    manipulation.jnt_ctrl.cmd_head(manipulation.jnt_ctrl.head_home_joint_values)

    # close the gripper
    manipulation.gpr_ctrl.close_right()

    # Move left arm to home with joint ctrl
    manipulation.jnt_ctrl.cmd_left_arm(manipulation.jnt_ctrl.left_arm_home_joint_values)

    manipulation.jnt_ctrl.cmd_torso([0.3])
    manipulation.jnt_ctrl.cmd_head([0.5, 0.5])

    # Move right arm to home in cartesian
    home_pose = GeometryPose()
    home_pose.position.x = 0.581
    home_pose.position.y = 0.003
    home_pose.position.z = 0.637

    home_pose.orientation.x = -0.5
    home_pose.orientation.y = -0.5
    home_pose.orientation.z = 0.5
    home_pose.orientation.w = 0.5

    handoff_pose = GeometryPose()
    handoff_pose.position.x = 0.807
    handoff_pose.position.y = 0.053
    handoff_pose.position.z = 0.978

    handoff_pose.orientation.x = -0.70710678
    handoff_pose.orientation.y = 0
    handoff_pose.orientation.z = 0
    handoff_pose.orientation.w = 0.70710678

    rospy.loginfo("Going to home orientation")
    manipulation.right_cartesian.cmd_orientation(home_pose.orientation)
    rospy.loginfo("Arrived at home orientation")

    for iter in range(3):
        rospy.loginfo("Going to home pose")
        manipulation.right_cartesian.cmd_position(home_pose.position, True)
        rospy.loginfo("Arrived at home pose")

        rospy.loginfo("Going to handoff pose")
        manipulation.right_cartesian.cmd_position(handoff_pose.position, True)
        rospy.loginfo("Arrived at handoff pose")
