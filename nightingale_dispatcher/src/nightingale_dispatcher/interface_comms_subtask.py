#!/usr/bin/env python

import rospy

from nightingale_dispatcher.subtask import Subtask
from nightingale_ros_bridge import RobotStatus, BridgeConfig, UserInputs

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
            res = self.interface_comms_proxy(self.robot_state)

            # state machine for interface responses
            # TODO replace with actual input codes
            if res.user_input == UserInputs.NO_INPUT:
                # do nothing, just updating patient status
                pass
            elif res.user_input == UserInputs.STOCK_ITEMS:
                # start up navigation task with goal of STOCK 
                pass
            elif res.user_input == UserInputs.DELIVER_ITEMS:
                # start up navigation task with goal of PATIENT ROOM
                pass
            elif res.user_input == UserInputs.RETURN_HOME:
                # start up navigation task with goal of HOME
                pass
            elif res.user_input == UserInputs.START_EXTEND_ARM:
                # start up manipulation task with arm goal of DELIVERY
                pass
            elif res.user_input == UserInputs.ITEMS_TAKEN:
                # intput to let node know the patient status is during arm delivery 
                pass
            elif res.user_input == UserInputs.START_RETRACT_ARM:
                # start up manipulation task with arm goal of ARM_HOME
                pass
            elif res.user_input == UserInputs.WD_TIMEOUT:
                # start up navigation task with goal of HOME. Quit other tasks related to patient
                pass

            return res.user_input

        except rospy.ServiceException as e:
            print(f"Interface communication failed service call failed: {e}")
            return self.SERVICE_ERROR


