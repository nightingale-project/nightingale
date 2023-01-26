#!/usr/bin/env python

import rospy

from nightingale_dispatcher.task import Task


class NavigateTask(Task):
    # Go to position with given context
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # Check if there are intermediate steps to travel to the destination
        # Call room_runner to drive to pose
        raise NotImplementedError()