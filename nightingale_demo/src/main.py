# /usr/bin/env python3

import rospy

from control_msgs.msg import FollowJointTrajectoryGoal
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig, RobotStatus
from nightingale_manipulation import ManipulationControl
from std_msgs.msg import Bool, String


class SymposiumDemo:
    def __init__(self):
        rospy.init_node("symposium_demo_node")

        self.arm_pub = rospy.Publisher(
            "/movo/right_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryGoal,
            queue_size=2,
        )

        self.right_collision_sub = rospy.Subscriber(
            "/nightingale/right_arm/collision", Bool, self.right_collision_cb
        )
        self.left_collision_sub = rospy.Subscriber(
            "/nightingale/left_arm/collision", Bool, self.left_collision_cb
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
            BridgeConfig.UPDATE_UI_SERVICE, String, queue_size=10
        )

        self.screen_button_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.screen_button_cb
        )

    def right_collision_cb(self, msg):
        if self.enable_right_collision and msg.data == True:
            self.collision_screen_pub.publish(String(f"{RobotStatus.ARM_COLLISION}"))

    def left_collision_cb(self, msg):
        if self.enable_left_collision and msg.data == True:
            self.collision_screen_pub.publish(String(f"{RobotStatus.ARM_COLLISON}"))

    def enable_right_collision_cb(self, msg):
        self.enable_right_collision = msg.data

    def enable_left_collision_cb(self, msg):
        self.enable_left_collision = msg.data


if __name__ == "__main__":
    main()
