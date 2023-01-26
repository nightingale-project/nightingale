#!/usr/bin/env python

import rospy


class IdleTask:
    # Goto idle position and wait
    # TODO define inputs
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self):
        # Goto home position
        raise NotImplementedError()