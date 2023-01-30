#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task import Task


class TriageTask(Task):
    TIMEOUT = 1    
    # Once at the patient bedside, wait for a user input on the tablet
    # TODO define parameters
    def __init__(self, *args, **kwargs):
        raise NotImplementedError()

    # TODO define inputs
    def execute(self, *args, **kwargs):
        # show user main screen. Blocks until dismiss or stock items call given
        userinput = self.interface_comms_client(RobotStatus.BEDSIDE_IDLE)
        if userinput == UserInputs.STOCK_ITEMS:
            # queue up go to stock items phase
            pass
        elif userinput == UserInputs.RETURN_HOME:
            # queue up go home phase
            pass
        elif userinput == UserInputs.WD_TIMEOUT:
            # queue up go home phase
            # not sure if different processing needed
            pass

        raise NotImplementedError()
