#!/usr/bin/env python

import rospy


class TriageTask:
    # Once at the patient bedside, wait for a user input on the tablet
    # TODO define inputs
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self):
        # show user main screen
        # wait for an item request, home command, or timeout
        raise NotImplementedError()