# /usr/bin/env python3

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from nightingale_ros_bridge.bridge_interface_config import (
    BridgeConfig,
    RobotStatus,
    UserInput,
)
from nightingale_manipulation import ManipulationControl
from std_msgs.msg import Bool, String


class SymposiumDemo:
    def __init__(self):
        rospy.init_node("symposium_demo_node")

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

    def screen_button_cb(self, msg):
        # goal = FollowJointTrajectoryGoal()
        # goal.path_tolerance = 1
        # goal.goal_tolerance = 1
        # goal.goal_time_tolerance = rospy.Duration(10)

        if msg.data == UserInput.START_EXTEND_ARM:
            rospy.loginfo(f"Start extending arm")
        #     goal.trajectory = []
        elif msg.data == UserInput.START_RETRACT_ARM:
            rospy.loginfo(f"Start retracting arm")
        #     goal.trajectory = []

        # self.arm_action_client.send_goal(goal)


if __name__ == "__main__":
    main()
