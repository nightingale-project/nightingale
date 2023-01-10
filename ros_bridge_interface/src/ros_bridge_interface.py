#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json


class RosBridgeInterface:
    def __init__(self):
        rospy.init_node('ros_bridge_interface', anonymous=True)

        self.interface_action_sub = rospy.Subscriber("interface/action", String, self.interface_action_callback)

        self.example_pub = rospy.Publisher("example/example", String)

    def interface_action_callback(self, msg):
        data = json.loads(msg.data)
        if data['action'] == 0:
            self.example_pub.publish(String("hello World"))


def main():
    rospy.spin()


if __name__ == '__main__':
    interface = RosBridgeInterface()
    main()
