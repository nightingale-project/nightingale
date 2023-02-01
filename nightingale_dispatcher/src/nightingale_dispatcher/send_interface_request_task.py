#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task
from nightingale_ros_bridge import BridgeConfig, UserInputs
from nightingale_msgs.srv import InterfaceCall


class SendInterfaceRequestTask(Task):
    def __init__(self):
        pass

    def execute(self, robot_state):
        rospy.wait_for_service(BridgeConfig.UPDATE_UI_SERVICE, InterfaceCall)
        try:
            # spin up proxy when needed
            self.interface_comms_proxy = rospy.ServiceProxy(
                BridgeConfig.UPDATE_UI_SERVICE, InterfaceCall
            )
            user_input = self.interface_comms_proxy(robot_state)
            # convert input to return code
            status = Task.SUCCESS
            if user_input == UserInputs.STOCK_ITEMS:
                status = Task.STOCK_ITEMS
            elif user_input == UserInputs.DELIVER_ITEMS:
                status = Task.DELIVER
            elif user_input == UserInputs.RETURN_HOME:
                status = Task.DISMISS
            elif user_input == UserInputs.WD_TIMEOUT:
                status = Task.WD_TIMEOUT
            elif user_input == UserInputs.NO_INPUT:
                # return success
                pass
            return status
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
