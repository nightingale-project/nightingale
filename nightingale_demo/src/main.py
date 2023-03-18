# /usr/bin/env python3

import actionlib
import rospy
import json
from playsound import playsound

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from nightingale_ros_bridge.bridge_interface_config import (
    BridgeConfig,
    RobotStatus,
    UserInputs,
)
from nightingale_manipulation.manipulation_action_client import ManipulationControl
from std_msgs.msg import Bool, Float64, String
from geometry_msgs.msg import Point


class SymposiumDemo:
    def __init__(self):
        rospy.init_node("symposium_demo_node")

        self.audio_file = rospy.get_param("/symposium_demo/audio_file")

        self.arm_action_client = actionlib.SimpleActionClient(
            "/movo/right_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        self.right_collision_sub = rospy.Subscriber(
            "/nightingale/right_arm/collision", Bool, self.right_collision_cb
        )
        self.left_collision_sub = rospy.Subscriber(
            "/nightingale/left_arm/collision", Bool, self.left_collision_cb
        )
        self.right_collision_threshold_pub = rospy.Publisher(
            "/nightingale/right_arm/collision_threshold", Float64, queue_size=10
        )
        self.enable_right_collision = True
        self.enable_left_collision = True
        self.enable_right_collision_sub = rospy.Subscriber(
            "/nightingale/right_arm/collision_enable",
            Bool,
            self.enable_right_collision_cb,
        )
        self.enable_left_collision_sub = rospy.Subscriber(
            "/nightingale/left_arm/collision_enable",
            Bool,
            self.enable_left_collision_cb,
        )
        self.collision_screen_pub = rospy.Publisher(
            BridgeConfig.ROBOT_STATUS_TOPIC, String, queue_size=10
        )

        self.screen_button_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.screen_button_cb
        )

        self.arm_control = ManipulationControl()
        #self.arm_control.home_left()
        self.arm_control.jnt_ctrl.cmd_right_arm(
            self.arm_control.jnt_ctrl.right_arm_home_joint_values
        )

    def right_collision_cb(self, msg):
        if self.enable_right_collision and msg.data == True:
            rospy.loginfo("Collision right")
            self.collision_screen_pub.publish(String(f"9"))

    def left_collision_cb(self, msg):
        if self.enable_left_collision and msg.data == True:
            rospy.loginfo("Collision left")
            self.collision_screen_pub.publish(String("9"))

    def enable_right_collision_cb(self, msg):
        self.enable_right_collision = msg.data

    def enable_left_collision_cb(self, msg):
        self.enable_left_collision = msg.data

    def chime(self):
        playsound(self.audio_file)

    def screen_button_cb(self, msg):
        dict_data = json.loads(msg.data)
        action = int(dict_data["action"])
        rospy.loginfo(f"action: {action}")
        self.chime()
        if action == UserInputs.START_EXTEND_ARM:
            status = self.arm_control.trajectory_inversion_server.fast_extend()
            rospy.loginfo(f"Extend {status}")
        elif action == UserInputs.START_RETRACT_ARM:
            status = self.arm_control.trajectory_inversion_server.fast_retract()
            rospy.loginfo(f"Retract {status}")
        elif action == 10:
            return self.arm_control.jnt_ctrl.home_right_arm()


if __name__ == "__main__":
    demo = SymposiumDemo()
    rospy.spin()
