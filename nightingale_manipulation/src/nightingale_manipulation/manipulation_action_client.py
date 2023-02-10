#!/usr/bin/env python3

import actionlib
import rospy
import math

# joint
from moveit_action_handlers.msg import PropertyValuePair
from moveit_action_handlers.msg import MoveToJointsMoveItAction
from moveit_action_handlers.msg import MoveToJointsMoveItGoal
from moveit_action_handlers.msg import MoveToJointsMoveItResult

from moveit_action_handlers.msg import ActionResultStatusConstants
from sensor_msgs.msg import JointState

# cartesian
from moveit_action_handlers.msg import MoveToPoseMoveItAction
from moveit_action_handlers.msg import MoveToPoseMoveItGoal
from moveit_action_handlers.msg import MoveToPoseMoveItResult
from moveit_action_handlers.msg import PoseStamped

# forward kinematics
from urdf_parser_py.urdf import Robot
import tf_conversions, tf2_ros
from geometry_msgs.msg import Pose as GeometryPose
from geometry_msgs.msg import Quaternion, Point

# TODO: replace with service call, requires refactoring of service to include gripper info
CFG = rospy.get_param("/nightingale_utils/joint_configurations")


# overriding the geometry msgs Pose class to add tolerance on equivalence check
class Pose(GeometryPose):
    def __init__(self, tolerance=0.01):
        super.__init__()
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


def quaternion_to_euler(q: Quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)
    orientation = Orientation(roll=roll_x, pitch=pitch_y, yaw=yaw_z)
    return orientation  # in radians


def joint_goal(joint_values, joint_names, eev=0.5, eea=0.5, timeout=10):
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
):
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

        # TODO: replace with service call
        self.left_arm_joint_names = CFG["left_arm_home"]["names"]
        self.right_arm_joint_names = CFG["right_arm_home"]["names"]
        self.left_arm_home_joint_values = CFG["left_arm_home"]["joints"]
        self.right_arm_home_joint_values = CFG["right_arm_home"]["joints"]

        self._right_joint_states = None
        self._left_joint_states = None
        self._joint_tolerance = joint_tolerance

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
            names_pos_dict = {}
            for i in range(len(names)):
                names_pos_dict[names[i]] = pos[i]

            self._right_joint_states = []
            self._left_joint_states = []

            # probably a redundant check but making sure the joint names exist from the /joint_states data
            if not (
                all(key in names_pos_dict.keys() for key in self.right_arm_joint_names)
                and all(
                    key in names_pos_dict.keys() for key in self.left_arm_joint_names
                )
            ):
                rospy.logerr(
                    "Invalid or missing joint name in data received from /joint_states"
                )
                return False

            for i in range(7):
                self._right_joint_states.append(
                    names_pos_dict[self.right_arm_joint_names[i]]
                )
                self._left_joint_states.append(
                    names_pos_dict[self.left_arm_joint_names[i]]
                )

            return True
        except rospy.ROSException:
            rospy.logwarn(
                "Unable to get data from /joint_states, wait_for_message timeout"
            )
            return False

    def get_joint_states(self):
        if self.update_joint_states():
            return {
                "left_arm": self._left_joint_states,
                "right_arm": self._right_joint_states,
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
        if not self.update_joint_states():
            return False

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
        return bool(error_count)

    def cmd_right_arm(self, joint_target: list, blocking=True) -> bool:
        """
        Commands movement of the right arm through the right_arm action server

        @param joint_target: list of target joint values
        @param blocking: default True, if blocking, function will wait for action server response
        @return: result of verify_joint_target and action server state if blocking
        """
        if self.verify_joint_target(joint_target, self._right_joint_states):
            goal = joint_goal(joint_target, self.right_arm_joint_names)
            self.right_arm.send_goal(goal)
            if blocking:
                self.right_arm.wait_for_result()
                status = self.right_arm.get_result().status
                if status == ActionResultStatusConstants.SUCCEEDED:
                    return True
                rospy.logwarn("cmd_right_arm failed with status: " + str(status))
                return False
            return True
        return False

    def cmd_left_arm(self, joint_target, blocking=True):
        """
        Commands the movement of the left arm through the left_arm action server

        @param joint_target: list of target joint values
        @param blocking: default True, if blocking, function will wait for action server response
        @return: boolean result of verify_joint_target and action server state if blocking
        """
        if self.verify_joint_target(joint_target, self._left_joint_states):
            goal = joint_goal(joint_target, self.left_arm_joint_names)
            self.left_arm.send_goal(goal)
            if blocking:
                self.left_arm.wait_for_result()
                status = self.left_arm.get_result().status
                if status == ActionResultStatusConstants.SUCCEEDED:
                    return True
                rospy.logwarn("cmd_left_arm failed with status: " + str(status))
                return False
            return True
        return False

    def home(self):
        self.cmd_left_arm(self.left_arm_home_joint_values, blocking=False)
        self.left_arm.wait_for_result()
        self.cmd_right_arm(self.right_arm_home_joint_values, blocking=False)
        self.right_arm.wait_for_result()
        return True  # TODO: action_server.get_result()


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
        if self.gripper_lock["right"]:
            self.right_gripper.send_goal(goal)
            if blocking:
                self.right_gripper.wait_for_result()
                return self.right_gripper.get_results()
            return True
        return False

    def cmd_left_gripper(self, goal, blocking=True):
        if self.gripper_lock["left"]:
            self.left_gripper.send_goal(goal)
            if blocking:
                self.left_gripper.wait_for_result()
                return self.left_gripper.get_results()
        return False

    def lock_right_gripper(self):
        self.gripper_lock["right"] = False

    def unlock_right_gripper(self):
        self.gripper_lock["right"] = True

    def lock_left_gripper(self):
        self.gripper_lock["left"] = False

    def unlock_left_gripper(self):
        self.gripper_lock["left"] = True


class ManipulationCartesianControl:
    def __init__(self, prefix):
        self.arm = actionlib.simple_action_client.SimpleActionClient(
            f"/moveit_action_handlers/{prefix}_arm/cartesian_ctrl",
            MoveToPoseMoveItAction,
        )
        self.arm.wait_for_server(timeout=10)
        self.prefix = prefix
        self.ee_ctrl_mode = 0
        self.orientation = Orientation()
        self.pose = Pose()
        self.ref_link = "base_link"
        self.ee_link = f"{prefix}_ee_link"
        # TODO: include correct home pose
        self.home_pose = None

        self.robot = Robot.from_parameter_server()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
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
                self.ref_link, self.ee_link, rospy.Time()
            )

            q = link_transform.transform.rotation
            self.orientation = quaternion_to_euler(q)
            p = link_transform.transform.translation
            self.pose.position = p
            self.pose.orientation = q
            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(f"{self.prefix} arm transformation lookup failed")
            return False

    def cmd_arm(self, pose: GeometryPose, blocking=True, ee_relative=False):
        """
        Commands the arm to move to a Pose in the cartesian space relative to the robot base_link

        @param pose: geometry_msgs.msg Pose
        @param blocking: if true, wait for action server result
        @param ee_relative: if true, use ee_link as base link
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
                pose.position, ref_link, self.orientation, mode=self.ee_ctrl_mode
            )

        # free end effector control mode
        else:
            goal = cartesian_goal(
                pose.position, ref_link, quaternion_to_euler(pose.orientation)
            )

        self.arm.send_goal(goal)
        if blocking:
            self.arm.wait_for_result()
            return self.arm.get_result()
        return True

    def set_fixed_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 1

    def set_free_ee_ctrl_mode(self):
        self.ee_ctrl_mode = 0

    def set_ref_link(self, ref):
        self.ref_link = ref

    def aproximate_home(self):
        self.set_ref_link("base_link")
        self.set_fixed_ee_ctrl_mode()
        return self.cmd_arm(self.home_pose)

    def get_pose(self):
        if not self.update_ee_pose():
            return False
        return self.pose

    def get_orientation(self):
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
        self.update_joint_states()
        rospy.loginfo("joint states received, initialization successful")

    def home(self):
        # home left arm in joint space and home right arm in cart or joint space
        # use self.jnt_ctrl.home() for now
        raise NotImplementedError()

    def extend_handoff(self):
        # extend right arm in cartesian space
        # TODO: replace with service call
        self.jnt_ctrl.cmd_right_arm(CFG["right_arm_extended_handoff"]["joints"])


# test code
if __name__ == "__main__":
    rospy.init_node("manipulation_control")
    manipulation = ManipulationControl()
    manipulation.jnt_ctrl.home()
