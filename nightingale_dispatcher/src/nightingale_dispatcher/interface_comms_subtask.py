#!/usr/bin/env python

import rospy

from nightingale_dispatcher.subtask import Subtask
from nightingale_ros_bridge import BridgeConfig 

class InterfaceCommsSubtask(Subtask):
    def __init__(self, name, robot_state, priority):
        super.__init__(name, priority)

        self.robot_state = robot_state

        # TODO Consider making a Singleton or share resources
        self.interface_comms_service_name = BridgeConfig.UPDATE_UI_SERVICE 
        rospy.wait_for_service(self.interface_comms_service_name)
        self.interface_comms_proxy = rospy.ServiceProxy(self.interface_comms_service_name, InterfaceCall)

    def execute(self):
        try:
            # extracts user input for task manager to handle
            res = self.interface_comms_proxy(self.robot_state)
            return res.user_input

        except rospy.ServiceException as e:
            print(f"Interface communication failed service call failed: {e}")
            return self.SERVICE_ERROR


