#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task_with_interface_comms import TaskWithInterfaceComms


class StockTask(TaskWithInterfaceComms):
    # Once arrived at the stock area, collect items for patient
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    def execute(self, *args, **kwargs):
        # raise arm with box and wait for user input
        # if fail return self.ERROR

        # show screen to load items
        user_input = self.update_interface_status(RobotStatus.ITEM_STOCK_REACHED)

        # retract arm after input
        # if fail return self.ERROR

        # send back information to decide what to do
        return user_input
