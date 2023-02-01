#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task_with_interface_comms import TaskWithInterfaceComms


class TriageTask(TaskWithInterfaceComms):
    # Once at the patient bedside, wait for a user input on the tablet
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self, *args, **kwargs):
        # show user main screen
        user_input = self.update_interface_state(RobotStatus.BEDSIDE_IDLE)
        return user_input
