#!/usr/bin/env python3

import actionlib
import rospy

# joint
from moveit_action_handlers.msg import (
    PropertyValuePair,
    MoveToJointsMoveItAction,
    MoveToJointsMoveItGoal,
)
from sensor_msgs.msg import JointState

# cartesian
from moveit_action_handlers.msg import (
    MoveToPoseMoveItAction,
    MoveToPoseMoveItGoal,
    PoseStamped,
)
from geometry_msgs.msg import Pose

# forward kinematics
# from urdf_parser_py.urdf import URDF
# from pykdl_utils.kdl_kinematics import KDLKinematics


def make_joint_goal(joint_values, joint_names, eev=0.5, eea=0.5, timeout=10):
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


def make_cartesian_goal(
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


class ManipulationJointControl:
    right_arm_joint_names = [
        "right_shoulder_pan_joint",
        "right_shoulder_lift_joint",
        "right_arm_half_joint",
        "right_elbow_joint",
        "right_wrist_spherical_1_joint",
        "right_wrist_spherical_2_joint",
        "right_wrist_3_joint",
    ]

    left_arm_joint_names = [
        "left_shoulder_pan_joint",
        "left_shoulder_lift_joint",
        "left_arm_half_joint",
        "left_elbow_joint",
        "left_wrist_spherical_1_joint",
        "left_wrist_spherical_2_joint",
        "left_wrist_3_joint",
    ]

    def __init__(self):
        self.left_arm_moveit_ah_name = "/moveit_action_handlers/left_arm/joint_ctrl"
        self.left_arm = actionlib.simple_action_client.SimpleActionClient(
            self.left_arm_moveit_ah_name, MoveToJointsMoveItAction
        )
        rospy.loginfo(f"Waiting for {self.left_arm_moveit_ah_name}")
        self.left_arm.wait_for_server()

        self.right_arm_moveit_ah_name = "/moveit_action_handlers/right_arm/joint_ctrl"
        self.right_arm = actionlib.simple_action_client.SimpleActionClient(
            self.right_arm_moveit_ah_name, MoveToJointsMoveItAction
        )
        rospy.loginfo(f"Waiting for {self.right_arm_moveit_ah_name}")
        self.right_arm.wait_for_server()

    def cmd_right_arm(self, joint_values, blocking=True):
        goal = make_joint_goal(joint_values, self.right_arm_joint_names)
        self.right_arm.send_goal(goal)
        if blocking:
            return self.right_arm.wait_for_result()
        return True

    def cmd_left_arm(self, joint_values, blocking=True):
        goal = make_joint_goal(joint_values, self.left_arm_joint_names)
        self.left_arm.send_goal(goal)
        if blocking:
            return self.left_arm.wait_for_result()
        return True


class ManipulationCartesianControl:
    def __init__(self):
        self.left_arm_moveit_ah_name = "/moveit_action_handlers/left_arm/cartesian_ctrl"
        self.left_arm = actionlib.simple_action_client.SimpleActionClient(
            self.left_arm_moveit_ah_name, MoveToPoseMoveItAction
        )
        rospy.loginfo(f"Waiting for {self.left_arm_moveit_ah_name}")
        self.left_arm.wait_for_server()

        self.right_arm_moveit_ah_name = (
            "/moveit_action_handlers/right_arm/cartesian_ctrl"
        )
        self.right_arm = actionlib.simple_action_client.SimpleActionClient(
            self.right_arm_moveit_ah_name, MoveToPoseMoveItAction
        )
        rospy.loginfo(f"Waiting for {self.right_arm_moveit_ah_name}")
        self.right_arm.wait_for_server()

        self.ee_ctrl_mode = 0

    def cmd_right_arm(self, pose: Pose, blocking=True):
        pass

    def cmd_left_arm(self, pose: Pose, blocking=True):
        pass

    def set_ee_ctrl_mode(self, mode):
        self.ee_ctrl_mode = mode


class ManipulationGripperControl:
    right_gripper_joint_names = [
        "right_gripper_finger1_joint",
        "right_gripper_finger2_joint",
        "right_gripper_finger3_joint",
    ]

    left_gripper_joint_names = [
        "left_gripper_finger1_joint",
        "left_gripper_finger2_joint",
        "left_gripper_finger3_joint",
    ]

    def __init__(self):
        self.left_gripper_moveit_ah_name = (
            "/moveit_action_handlers/left_arm/gripper_ctrl"
        )
        self.left_gripper = actionlib.simple_action_client.SimpleActionClient(
            self.left_gripper_moveit_ah_name, MoveToJointsMoveItAction
        )
        rospy.loginfo(f"Waiting for service {self.left_gripper_moveit_ah_name}")
        self.left_gripper.wait_for_server()

        self.right_gripper_moveit_ah_name = (
            "/moveit_action_handlers/right_arm/gripper_ctrl"
        )
        self.right_gripper = actionlib.simple_action_client.SimpleActionClient(
            self.right_gripper_moveit_ah_name, MoveToJointsMoveItAction
        )
        rospy.loginfo(f"Waiting for service {self.right_gripper_moveit_ah_name}")
        self.right_gripper.wait_for_server()

        self.right_open_goal = make_joint_goal(
            [0.23, 0.23, 0.23], right_gripper_joint_names
        )
        self.right_closed_goal = make_joint_goal([0, 0, 0], right_gripper_joint_names)

        self.left_open_goal = make_joint_goal(
            [0.23, 0.23, 0.23], left_gripper_joint_names
        )
        self.left_closed_goal = make_joint_goal([0, 0, 0], left_gripper_joint_names)

        self.gripper_lock = {"left": True, "right": True}
        self.unlock_left_gripper()
        self.unlock_right_gripper()

    def cmd_right_gripper(self, goal, blocking=True):
        if self.gripper_lock["right"]:
            self.right_gripper.send_goal(goal)
            if blocking:
                return self.right_gripper.wait_for_result()
            return True
        return False

    def cmd_left_gripper(self, goal, blocking=True):
        if self.gripper_lock["left"]:
            self.left_gripper.send_goal(goal)
            if blocking:
                return self.left_gripper.wait_for_result()
            return True
        return False

    def lock_right_gripper(self):
        self.gripper_lock["right"] = True

    def unlock_right_gripper(self):
        self.gripper_lock["right"] = False

    def lock_left_gripper(self):
        self.gripper_lock["left"] = True

    def unlock_left_gripper(self):
        self.gripper_lock["left"] = False


class ManipulationControl:
    def __init__(self):
        self.jnt_ctrl = ManipulationJointControl()
        rospy.loginfo("initialized joint ctrl")
        self.gpr_ctrl = ManipulationGripperControl()
        rospy.loginfo("initialized gripper ctrl")
        self.crt_ctrl = ManipulationCartesianControl()
        rospy.loginfo("initialized cartesian ctrl")

        try:
            self.joint_configuration_service = rospy.ServiceProxy(
                "/nightingale/robot_configuration_lookup", RobotConfigurationLookup
            )
            self.left_arm_home_joint_values = self.joint_configuration_service(
                "home", "left_arm"
            ).joints
            self.right_arm_home_joint_values = self.joint_configuration_service(
                "home", "right_arm"
            ).joints

            self.right_arm_handoff_joint_values = self.joint_configuration_service(
                "handoff", "right_arm"
            ).joints
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

            self.left_arm_home_joint_values = [
                1.5331797090297457,
                1.605558035871483,
                0.722653222587827,
                2.606298339748302,
                -2.3331829141612968,
                1.825435539687259,
                -2.9605818553548513,
            ]

            self.right_arm_home_joint_values = [
                -1.189840720238987,
                -1.8304754388921793,
                -0.31277335632770153,
                -2.367960136151233,
                0.9231204765356429,
                1.534414350511106,
                2.8363235525817165,
            ]

            self.right_arm_handoff_joint_values = [
                -1.497173771091874,
                -0.05384432355144005,
                -0.0472769683033043,
                -1.4575817535881566,
                3.1601009435928535,
                1.2459021253979168,
                1.515535734175753,
            ]

    def home(self):
        left_status = self.jnt_ctrl.cmd_left_arm(
            self.left_arm_home_joint_values, blocking=False
        )
        right_status = self.jnt_ctrl.cmd_right_arm(
            self.right_arm_home_joint_values, blocking=False
        )

        return left_status and right_status

    def handoff(self):
        self.jnt_ctrl.cmd_right_arm(self.right_arm_handoff_joint_values)


# test code
if __name__ == "__main__":
    rospy.init_node("manipulation_control")
    manipulation = ManipulationControl()
    manipulation.jnt_ctrl.home()
