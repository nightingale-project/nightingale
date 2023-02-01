#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task import Task


class TriageTask(Task):
    TIMEOUT = 1

    # Once at the patient bedside, wait for a user input on the tablet
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # show user main screen
        # wait for an item request, home command, or timeout
        raise NotImplementedError()
