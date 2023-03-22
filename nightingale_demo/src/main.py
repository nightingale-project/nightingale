# /usr/bin/env python3

import actionlib
import rospy
import json
from playsound import playsound
import numpy as np

from movo_msgs.msg import PanTiltCmd

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

        self.head_pub = rospy.Publisher("/movo/head/cmd", PanTiltCmd, queue_size=10)

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
        # self.arm_control.home_left()
        self.arm_control.jnt_ctrl.cmd_right_arm(
            self.arm_control.jnt_ctrl.right_arm_home_joint_values
        )

        self.arm_control.open_right_gripper()
        rospy.sleep(2.0)
        self.arm_control.close_right_gripper()

        # set last action to home
        self.arm_at_home = True

    def right_collision_cb(self, msg):
        if self.enable_right_collision == True and msg.data == True:
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

    def look_around(self):
        msg = PanTiltCmd()
        msg.pan_cmd.pos_rad = np.radians(40)
        msg.pan_cmd.vel_rps = 0.2
        msg.tilt_cmd.pos_rad = np.radians(10)
        msg.tilt_cmd.vel_rps = 0.2 / 4  # to travel in straight line
        self.head_pub.publish(msg)
        rospy.sleep(5)
        msg.pan_cmd.pos_rad = np.radians(-40)
        msg.pan_cmd.vel_rps = 0.2
        msg.tilt_cmd.pos_rad = np.radians(10)
        msg.tilt_cmd.vel_rps = 0
        self.head_pub.publish(msg)
        rospy.sleep(8)
        msg.pan_cmd.pos_rad = np.radians(0)
        msg.pan_cmd.vel_rps = 0.2
        msg.tilt_cmd.pos_rad = np.radians(0)
        msg.tilt_cmd.vel_rps = 0.2 / 4  # to travel in straight line
        self.head_pub.publish(msg)
        rospy.sleep(3)

    def screen_button_cb(self, msg):
        idle = rospy.get_param("/symposium_demo/idle", False)
        if idle:
            rospy.loginfo("screen button pressed. setting state to not idle")
            rospy.set_param("/symposium_demo/idle", False)
        dict_data = json.loads(msg.data)
        action = int(dict_data["action"])
        rospy.loginfo(f"action: {action}")

        self.enable_right_collision = False
        self.enable_left_collision = False
        if action == UserInputs.START_EXTEND_ARM and self.arm_at_home:
            self.chime()
            status = self.arm_control.trajectory_inversion_server.fast_extend()
            rospy.loginfo(f"Extend {status}")
            self.arm_at_home = False
        elif action == UserInputs.START_RETRACT_ARM and not self.arm_at_home:
            self.chime()
            status = self.arm_control.trajectory_inversion_server.fast_retract()
            rospy.loginfo(f"Retract {status}")
            self.arm_at_home = True
            idle = rospy.get_param("/symposium_demo/idle", False)
            if not idle:
                rospy.loginfo("arm retracted. now setting state to idle")
                rospy.set_param("/symposium_demo/idle", True)
        elif action == 10:
            self.chime()
            status = self.arm_control.jnt_ctrl.home_right_arm()
            rospy.loginfo(f"Home {status}")
            self.arm_at_home = True
        
        self.enable_right_collision = True
        self.enable_left_collision = True

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(10)
            idle = rospy.get_param("/symposium_demo/idle", False)
            if idle:
                total_wait_time = 3 * 60 # 3 minutes
                step_time = 10
                for time_remaining in range(total_wait_time, 0, -step_time):
                    idle = rospy.get_param("/symposium_demo/idle", False)
                    if not idle:
                        rospy.loginfo(
                            "Robot set to not idle. Robot will not move head until idle state is restored"
                        )
                        break
                    rospy.loginfo(
                        f"{time_remaining} seconds before head starts looking around"
                    )
                    rospy.sleep(step_time)
                idle = rospy.get_param("/symposium_demo/idle", False)
                if idle:
                    rospy.loginfo(
                        "The demo mode is currently idle. We chime and look around."
                    )
                    self.chime()
                    self.look_around()
            else:
                rospy.loginfo(
                    "The demo mode is currently not idle. We dont look around."
                )


if __name__ == "__main__":
    demo = SymposiumDemo()
    demo.run()
