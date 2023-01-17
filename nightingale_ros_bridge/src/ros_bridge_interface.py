#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
import time

#from bridge_interface_config import robot_status, bridge_config
from nightingale_srvs.srv import (InterfaceCall, InterfaceCallResponse)


class RosBridgeInterface:
    def __init__(self):
        rospy.init_node('ros_bridge_interface_server', anonymous=True)
        self.interface_input_sub = rospy.Subscriber(bridge_config.USER_INPUT_TOPIC, String, self.interface_input_callback)
        self.set_screen_pub = rospy.Publisher(bridge_config.ROBOT_STATUS_TOPIC, String, queue_size=10)

    def interface_input_callback(self, msg):
        data = json.loads(msg.data)
        if data['action'] == -1:
            rospy.loginfo("set screen request received")
            time.sleep(1)
            self.set_screen_pub.publish(String(""))
            return True
        rospy.loginfo(data['action'])
        return False

    def interface_call(self, status):
        print(f"Received status {status}")
        # publish to UI tablet

        # wait for next response on USER_INPUT_TOPIC and return it
        return 1

    def main(self):
        rospy.loginfo("Starting Ros Bridge Interface...")
        rospy.spin()


def main():
    interface = RosBridgeInterface()
    interface.main()


if __name__ == '__main__':
    main()
