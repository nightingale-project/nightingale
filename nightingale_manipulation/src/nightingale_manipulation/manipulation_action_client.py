#!/usr/bin/env python3

import actionlib
import rospy
import np

# joint
from moveit_action_handlers.msg import PropertyValuePair
from moveit_action_handlers.msg import MoveToJointsMoveItAction
from moveit_action_handlers.msg import MoveToJointsMoveItGoal

from sensor_msgs.msg import JointState

# cartesian
from moveit_action_handlers.msg import MoveToPoseMoveItAction
from moveit_action_handlers.msg import MoveToPoseMoveItGoal
from moveit_action_handlers.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Pose

# forward kinematics
# from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_kinematics import KDLKinematics


# TODO: replace with service call, requires refactoring of service to include gripper info


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
    x, y, z, ref_link, roll=0, pitch=0, yaw=0, eev=0.5, eea=0.5, mode=0, timeout=20
):
    goal = MoveToPoseMoveItGoal()
    goal.constraint_mode = mode
    goal.endEffectorVelocity = eev
    goal.endEffectorAcceleration = eea
    goal.timeoutSeconds = timeout
    target_pose = PoseStamped()
    target_pose.header.frame_id = ref_link
    target_pose.pose.position.x = x
    target_pose.pose.position.y = y
    target_pose.pose.position.z = z
    target_pose.pose.orientation.roll = roll
    target_pose.pose.orientation.pitch = pitch
    target_pose.pose.orientation.yaw = yaw
    goal.target_pose = target_pose
    return goal


def get_head_angles(head_point):
    """
    :param: head_point PointStamped object
    :return: angles in xy and xz plane to align with point
    """
    # 2D unit vector that represents axis in both
    # XY and XZ planes
    axis_u_vect = np.array([1, 0])
    head_xy_vect = np.array([head_point.point.x, head_point.point.y])
    head_xz_vect = np.array([head_point.point.x, head_point.point.z])
    # return euler angles: alpha - XY, gamma - XZ
    alpha = np.arccos(
        np.dot(axis_u_vect, head_xy_vect)
        / (np.linalg.norm(head_xy_vect) * np.linalg.norm(axis_u_vect))
    )
    gamma = np.arccos(
        np.dot(axis_u_vect, head_xz_vect)
        / (np.linalg.norm(head_xz_vect) * np.linalg.norm(axis_u_vect))
    )
    return [alpha, gamma]


class ManipulationJointControl:
    def __init__(self):
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

    def cmd_right_arm(self, joint_values, blocking=True):
        goal = joint_goal(joint_values, self.right_arm_joint_names)
        self.right_arm.send_goal(goal)
        if blocking:
            return self.right_arm.wait_for_result()  # TODO: action_server.get_result()
        return True

    def cmd_left_arm(self, joint_values, blocking=True):
        goal = joint_goal(joint_values, self.left_arm_joint_names)
        self.left_arm.send_goal(goal)
        if blocking:
            return self.left_arm.wait_for_result()  # TODO: action_server.get_result()
        return True

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
                return self.right_gripper.wait_for_result()
            return True  # TODO: action_server.get_result()
        return False

    def cmd_left_gripper(self, goal, blocking=True):
        if self.gripper_lock["left"]:
            self.left_gripper.send_goal(goal)
            if blocking:
                return self.left_gripper.wait_for_result()
            return True  # TODO: action_server.get_result()
        return False

    def lock_right_gripper(self):
        self.gripper_lock["right"] = True

    def unlock_right_gripper(self):
        self.gripper_lock["right"] = False

    def lock_left_gripper(self):
        self.gripper_lock["left"] = True

    def unlock_left_gripper(self):
        self.gripper_lock["left"] = False


class ManipulationCartesianControl:
    def __init__(self):
        self.right_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/right_arm/cartesian_ctrl", MoveToPoseMoveItAction
        )
        self.right_arm.wait_for_server()

        self.left_arm = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/left_arm/cartesian_ctrl", MoveToPoseMoveItAction
        )
        self.left_arm.wait_for_server()
        self.ee_ctrl_mode = 0

    def cmd_right_arm(self, pose: Pose, blocking=True):
        raise NotImplementedError()

    def cmd_left_arm(self, pose: Pose, blocking=True):
        raise NotImplementedError()

    def set_ee_ctrl_mode(self, mode):
        self.ee_ctrl_mode = mode


class ManipulationControl:
    def __init__(self):
        rospy.loginfo("starting manipulation initialization, waiting for servers...")
        self.jnt_ctrl = ManipulationJointControl()
        rospy.loginfo("initialized joint ctrl")
        self.gpr_ctrl = ManipulationGripperControl()
        rospy.loginfo("initialized gripper ctrl")
        self.crt_ctrl = ManipulationCartesianControl()
        rospy.loginfo("initialized cartesian ctrl")

        self.right_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self.left_joint_states = [0, 0, 0, 0, 0, 0, 0]
        self.linear_joint_state = [0]
        self.right_ee_pose = None
        self.left_ee_pose = None
        self.update_joint_states()
        rospy.loginfo("joint states received, initialization successful")

    def update_joint_states(self):
        joint_states = rospy.client.wait_for_message("/joint_states", JointState)
        p = joint_states.position
        self.right_joint_states = [p[14], p[13], p[10], p[11], p[15], p[16], p[17]]
        self.left_joint_states = [p[4], p[3], p[0], p[1], p[5], p[6], p[7]]
        self.linear_joint_state = [p[8]]

    def update_ee_pose(self):
        """
        robot_urdf = URDF.from_xml_string("urdf_str")
        self.update_joint_states()
        # base -> right ee
        kdl_kin = KDLKinematics(robot_urdf, "base_link", "right_ee_link")
        self.right_ee_pose = kdl_kin.forward(self.linear_joint_state + self.right_joint_states)

        # base -> left ee
        kdl_kin = KDLKinematics(robot_urdf, "base_link", "left_ee_link")
        self.left_ee_pose = kdl_kin.forward(self.linear_joint_state + self.left_joint_states)
        """
        raise NotImplementedError()

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
