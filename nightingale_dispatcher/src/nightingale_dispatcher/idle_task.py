#!/usr/bin/env python

import rospy

from nightingale_dispatcher.task import Task


class IdleTask(Task):
    # Goto idle position and wait
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # Goto home position
        raise NotImplementedError()