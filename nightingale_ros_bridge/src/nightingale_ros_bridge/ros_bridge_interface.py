#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import json
import time

from nightingale_ros_bridge.bridge_interface_config import RobotStatus, BridgeConfig
from nightingale_msgs.srv import (InterfaceCall, InterfaceCallResponse)

class RosBridgeInterface:
    def __init__(self):
        rospy.init_node('ros_bridge_interface_server', anonymous=True)
        self.interface_input_sub = rospy.Subscriber(BridgeConfig.USER_INPUT_TOPIC, String, self.interface_input_cb)
        self.set_robot_status_pub = rospy.Publisher(BridgeConfig.ROBOT_STATUS_TOPIC, String, queue_size=10)

        # service for M.P and UI comms
        interface_service = rospy.Service(BridgeConfig.UPDATE_UI_SERVICE, InterfaceCall, self.interface_service_cb)
        self.new_input = -1

    # callback for when tablet recieves user input
    def interface_input_cb(self, msg):
        data = json.loads(msg.data)
        self.new_input = data
        if data['action'] == -1:
            rospy.loginfo("User input received")
            time.sleep(1)
            return True
        rospy.loginfo(data['action'])
        return False

    # callback on M.P service call
    def interface_service_cb(self, status):
        print(f"Received status {status}")
        # publish to UI tablet
        self.set_robot_status_pub.publish(String(""))

        # wait for next response on USER_INPUT_TOPIC and return it
        # blocks M.P but should be ok since the robot should be doing nothing but
        # waiting on user input
        while self.new_input == -1:
            # wait until user input
            time.sleep(0.1)

        new_response = self.new_input

        # reset flag for next input
        self.new_input = -1

        # respond
        return new_response 

    def main(self):
        rospy.loginfo("Starting Ros Bridge Interface...")
        rospy.spin()


def main():
    interface = RosBridgeInterface()
    interface.main()


if __name__ == '__main__':
    main()
