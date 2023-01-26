#!/usr/bin/env python

import rospy


class NavigateTask:
    # Go to position with given context
    def __init__(self):
        raise NotImplementedError()

    def execute(self):
        raise NotImplementedError()