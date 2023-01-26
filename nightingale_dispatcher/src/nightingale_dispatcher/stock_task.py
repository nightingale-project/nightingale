#!/usr/bin/env python

import rospy


class StockTask:
    # Once arrived at the stock area, collect items for patient
    def __init__(self):
        raise NotImplementedError()

    def execute(self):
        # show screen to load items
        # load items into the box
        # lower arms to driving pose
        raise NotImplementedError()