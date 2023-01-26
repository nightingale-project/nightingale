#!/usr/bin/env python

import rospy


class HandoffTask:
    # Once arrived at the bedside, pass of items to patient
    # TODO define inputs
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self):
        # look for patient
        # show screen
        # move box to patient
        raise NotImplementedError()