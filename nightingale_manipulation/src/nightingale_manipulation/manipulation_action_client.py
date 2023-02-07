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

from nightingale_msgs.srv import RobotConfigurationLookup


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
        rospy.loginfo("Starting manipulation initialization, waiting for servers...")
        self.jnt_ctrl = ManipulationJointControl()
        rospy.loginfo("Initialized joint ctrl")
        self.gpr_ctrl = ManipulationGripperControl()
        rospy.loginfo("Initialized gripper ctrl")
        self.crt_ctrl = ManipulationCartesianControl()
        rospy.loginfo("Initialized cartesian ctrl")

        robot_config_service = "/nightingale/robot_configuration_lookup"
        rospy.wait_for_service(robot_config_service)
        lookup_client = rospy.ServiceProxy(
            robot_config_service, RobotConfigurationLookup
        )
        rospy.loginfo(
            "Nightingale Manipulation Control found robot configuration lookup service server",
            logger_name=self.logger_name,
        )
        try:
            response = lookup_client(RobotConfigurationLookup.LEFT_ARM, "home")
            self.left_arm_joint_names = response.jnt_states.names
            self.left_arm_home_joint_values = response.jnt_states.position

            response = lookup_client(RobotConfigurationLookup.RIGHT_ARM, "home")
            self.right_arm_joint_names = response.jnt_states.names
            self.right_arm_home_joint_values = response.jnt_states.position

            response = lookup_client(RobotConfigurationLookup.RIGHT_ARM, "handoff")
            self.right_arm_handoff_joint_values = response.jnt_states.position

            response = lookup_client(
                RobotConfigurationLookup.LEFT_GRIPPER, "open_gripper"
            )
            self.left_gripper_joint_names = response.jnt_states.names
            self.left_gripper_open_joint_values = response.jnt_states.position

            response = lookup_client(
                RobotConfigurationLookup.LEFT_GRIPPER, "closed_gripper"
            )
            self.left_gripper_closed_joint_values = response.jnt_states.position

            response = lookup_client(
                RobotConfigurationLookup.RIGHT_GRIPPER, "open_gripper"
            )
            self.right_gripper_joint_names = response.jnt_states.names
            self.right_gripper_open_joint_values = response.jnt_states.position

            response = lookup_client(
                RobotConfigurationLookup.RIGHT_GRIPPER, "closed_gripper"
            )
            self.right_gripper_closed_joint_values = response.jnt_states.position

        except rospy.ServiceException as exc:
            rospy.logerr(
                "Nightingale Manipulation Control failed to call robot configuration lookup service",
                logger_name=self.logger_name,
            )

        self.joint_states_sub = rospy.Subscriber(
            "/joint_states", JointState, self.update_joint_states
        )

    def update_joint_states(self):
        raise NotImplementedError()

    def home(self):
        # home left arm in joint space and home right arm in cart or joint space
        self.jnt_ctrl.cmd_left_arm(self.left_arm_home_joint_values)
        self.jnt_ctrl.cmd_right_arm(self.right_arm_home_joint_values)

    def extend_handoff(self):
        # extend right arm in cartesian space
        self.jnt_ctrl.cmd_right_arm(self.left_arm_handoff_joint_values)

    def open_left_gripper(self):
        self.gpr_ctrl.cmd_left_gripper(self.left_gripper_open_joint_values)

    def open_right_gripper(self):
        self.gpr_ctrl.cmd_right_gripper(self.right_gripper_open_joint_values)

    def close_left_gripper(self):
        self.gpr_ctrl.cmd_left_gripper(self.left_gripper_open_joint_values)

    def close_right_gripper(self):
        self.gpr_ctrl.cmd_right_gripper(self.right_gripper_open_joint_values)


# test code
if __name__ == "__main__":
    rospy.init_node("manipulation_control")
    manipulation = ManipulationControl()
    manipulation.open_right_gripper()
    manipulation.home()
    manipulation.close_right_gripper()
