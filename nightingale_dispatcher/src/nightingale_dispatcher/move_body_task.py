#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_body.body_control import BodyControl


class MoveBodyTask(Task):
    def __init__(self):
        self.body_ctrl = BodyControl()

    def home(self):
        self.body_ctrl.home()
        if self.body_ctrl.home():
            return TaskCodes.SUCCESS
        else:
            return TaskCodes.ERROR

    def handoff(self):
        if self.body_ctrl.handoff():
            return TaskCodes.SUCCESS
        else:
            return TaskCodes.ERROR
