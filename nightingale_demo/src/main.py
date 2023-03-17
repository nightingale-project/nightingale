# /usr/bin/env python3

import rospy

from std_msgs.msg import Bool, String
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig, RobotStatus


class SymposiumDemo:
    def __init__(self):
        rospy.init_node("symposium_demo_node")

        self.right_collision_sub = rospy.Subscriber(
            "/nightingale/right_arm/collision", Bool, self.collision_cb
        )
        self.left_collision_sub = rospy.Subscriber(
            "/nightingale/left_arm/collision", Bool, self.collision_cb
        )
        self.collision_screen_pub = rospy.Publisher(
            BridgeConfig.UPDATE_UI_SERVICE, String, queue_size=10
        )

    def collision_cb(self, msg):
        if msg.data == True:
            self.collision_screen_pub.publish(String(f"{RobotStatus.ARM_COLLISION}"))


if __name__ == "__main__":
    main()
