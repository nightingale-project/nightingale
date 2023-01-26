#!/usr/bin/env python

import rospy


class TriageTask:
    # Once at the patient bedside, wait for a user input on the tablet
    def __init__(self):
        raise NotImplementedError()

    def execute(self):
        # show user main screen
        # wait for an item request, home command, or timeout
        raise NotImplementedError()