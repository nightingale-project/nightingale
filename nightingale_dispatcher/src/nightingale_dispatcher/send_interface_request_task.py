#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task


class SendInterfaceRequestTask(Task):
    def __init__(self):
        pass

    def execute(self, robot_state):
        user_input = self.interface_comms_request()
        return user_input
