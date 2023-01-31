#!/usr/bin/env python3
import rospy
from nightingale_ros_bridge import BridgeConfig, RobotStatus, UserInputs
from nightingale_msgs.srv import InterfaceCall


class Task:
    def __init__(self):
        self.ERROR = -1
        self.SUCCESS = 0
        self.interface_comms_service_name = BridgeConfig.UPDATE_UI_SERVICE

    def interface_comms_client(self, robot_state):
        rospy.wait_for_service(self.interface_comms_service_name, InterfaceCall)
        try:
            # spin up proxy when needed
            self.interface_comms_proxy = rospy.ServiceProxy(
                self.interface_comms_service_name, InterfaceCall
            )
            user_input = self.interface_comms_proxy(robot_state)
            return user_input
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
