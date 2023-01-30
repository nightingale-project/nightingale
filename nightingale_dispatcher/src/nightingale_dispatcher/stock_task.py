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
        # called after stock room reached
        # extend arm
        status = extend_arm_action
        if status != SUCCESS:
            # handle error
            pass 

        # interface call to set stock items
        userinput = self.interface_comms_client(RobotStatus.ITEM_STOCK_REACHED)

        # upon stock finished or cancel retract arm
        status = retract_arm_action
        if status != SUCCESS:
            # handle error
            # try arm retract again
            pass 

        if userinput == UserInputs.DELIVER_ITEMS:
            # queue up navigate to patient phase
            pass
        elif userinput == UserInputs.RETURN_HOME:
            # queue up navigate to home phase
            pass
        
        raise NotImplementedError()
