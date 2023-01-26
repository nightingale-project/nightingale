#!/usr/bin/env python

import rospy


class NavigateTask:
    # Go to position with given context
    # TODO define inputs
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self):
        # Check if there are intermediate steps to travel to the destination
        # Call room_runner to drive to pose
        raise NotImplementedError()