#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
import time


class RosBridgeInterface:
    def __init__(self):
        rospy.init_node('ros_bridge_interface_server', anonymous=True)
        self.interface_action_sub = rospy.Subscriber("ui/robot/call_action", String, self.interface_action_callback)
        self.set_screen_pub = rospy.Publisher("ui/app/set_screen", String, queue_size=10)

    def interface_action_callback(self, msg):
        data = json.loads(msg.data)
        if data['action'] == -1:
            rospy.loginfo("set screen request received")
            time.sleep(1)
            self.set_screen_pub.publish(String(""))
            return True
        rospy.loginfo(data['action'])
        return False

    def main(self):
        rospy.loginfo("Starting Ros Bridge Interface...")
        rospy.spin()


def main():
    interface = RosBridgeInterface()
    interface.main()


if __name__ == '__main__':
    main()
