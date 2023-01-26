#!/usr/bin/env python

import rospy


class IdleTask:
    # Goto idle position and wait
    def __init__(self):
        raise NotImplementedError()

    def execute(self):
        # Goto home position
        raise NotImplementedError()