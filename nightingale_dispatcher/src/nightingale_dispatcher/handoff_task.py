#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task_with_interface_comms import TaskWithInterfaceComms


class HandoffTask(TaskWithInterfaceComms):
    # Once arrived at the bedside, pass of items to patient
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # look for patient
        # show screen
        # move box to patient
        raise NotImplementedError()
