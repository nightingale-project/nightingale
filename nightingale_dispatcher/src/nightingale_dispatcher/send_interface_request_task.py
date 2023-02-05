#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig, UserInputs
from nightingale_msgs.srv import InterfaceCall


class SendInterfaceRequestTask(Task):
    def __init__(self):
        pass

    def execute(self, robot_state):
        try:
            rospy.loginfo("Waiting for interface service")
            # returns exception if failed
            rospy.wait_for_service(BridgeConfig.UPDATE_UI_SERVICE, timeout=10)

            rospy.loginfo("Got interface service")
            # spin up proxy when needed
            self.interface_comms_proxy = rospy.ServiceProxy(
                BridgeConfig.UPDATE_UI_SERVICE, InterfaceCall
            )
            interface_response = self.interface_comms_proxy(robot_state)
            input_code = interface_response.user_input
            rospy.loginfo(f"USER INPUT {input_code}")
            # convert input to return code
            status = Task.SUCCESS
            if input_code == UserInputs.STOCK_ITEMS:
                status = Task.STOCK_ITEMS
            elif input_code == UserInputs.DELIVER_ITEMS:
                status = Task.DELIVER_ITEMS
            elif input_code == UserInputs.RETURN_HOME:
                status = Task.DISMISS
            elif input_code == UserInputs.WD_TIMEOUT:
                status = Task.WD_TIMEOUT
            elif input_code == UserInputs.NO_INPUT:
                # return success
                pass
            return status
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
