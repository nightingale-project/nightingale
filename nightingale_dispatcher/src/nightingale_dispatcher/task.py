#!/usr/bin/env python3
import rospy
from nightingale_ros_bridge import BridgeConfig, RobotStatus, UserInputs


class Task:
    def __init__(self):
        self.ERROR = -1
        self.SUCCESS = 0
        # client for interface communication
        self.current_robot_state = RobotStatus.IDLE_HOME
        self.interface_comms_service_name = BridgeConfig.UPDATE_UI_SERVICE
        rospy.wait_for_service(self.interface_comms_service_name)
        self.interface_comms_proxy = rospy.ServiceProxy(
            self.interface_comms_service_name, InterfaceCall
        )


     # helper functions
     def process_user_input(self, user_input):
        """
        takes in the enum of user input codes and returns an enum based on what robot state should come next and what task is to be queued
        :param user_input: Enum of user inputs
        :return next_robot_state: Enum of next robot state
        """
        next_robot_state = self.current_robot_state
        # state machine for interface responses
        if user_input == UserInputs.NO_INPUT:
            # do nothing, just updating robot status on interface
            pass
        elif user_input == UserInputs.STOCK_ITEMS:
            # start up navigation task with goal of STOCK
            next_robot_state = RobotStatus.STOCK_ITEMS
        elif user_input == UserInputs.DELIVER_ITEMS:
            # start up navigation task with goal of PATIENT ROOM
            next_robot_state = RobotStatus.DELIVER_ITEMS
        elif user_input == UserInputs.RETURN_HOME:
            # start up navigation task with goal of HOME
            next_robot_state = RobotStatus.DRIVING
        elif user_input == UserInputs.START_EXTEND_ARM:
            # start up manipulation task with arm goal of DELIVERY
            next_robot_state = RobotStatus.EXTENDING_ARM
        elif user_input == UserInputs.ITEMS_TAKEN:
            # intput to let node know the patient status is during arm delivery
            next_robot_state = RobotStatus.ARM_EXTENDED
        elif user_input == UserInputs.START_RETRACT_ARM:
            # start up manipulation task with arm goal of ARM_HOME
            next_robot_state = RobotStatus.RETRACTING_ARM
        elif user_input == UserInputs.WD_TIMEOUT:
            # start up navigation task with goal of HOME. Quit other tasks related to patient
            next_robot_state = RobotStatus.DRIVING

        return next_robot_state


