#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task import Task


class StockTask(Task):
    # Once arrived at the stock area, collect items for patient
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # show screen to load items
        # load items into the box
        # lower arms to driving pose
        raise NotImplementedError()