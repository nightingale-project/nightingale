#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json


class RosBridgeInterface:
    def __init__(self):
        rospy.init_node('ros_bridge_interface_server', anonymous=True)
        self.interface_action_sub = rospy.Subscriber("interface/action", String, self.interface_action_callback)
        self.example_pub = rospy.Publisher("example/example", String, queue_size=10)

    def interface_action_callback(self, msg):
        data = json.loads(msg.data)
        if data['action'] == -1:
            self.example_pub.publish(String("hello World"))
            return True
        print(data['action'])

    def main(self):
        rospy.loginfo("Starting Ros Bridge Interface...")
        rospy.spin()


def main():
    interface = RosBridgeInterface()
    interface.main()


if __name__ == '__main__':
    main()
