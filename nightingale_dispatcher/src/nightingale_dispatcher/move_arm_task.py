#!/usr/bin/env python3

import rospy
import actionlib

from nightingale_dispatcher.task import Task
from actionlib_msgs.msg import GoalStatus

from nightingale_manipulation.manipulation_action_client import ManipulationControl


class MoveArmTask(Task):
    def __init__(self):
        self.manipulation_client = ManipulationControl()

    def execute(self, end_pose):
        if end_pose == "home":
            self.manipulation_client.home()
        elif end_pose == "handoff":
            self.manipulation_client.handoff()
        else:
            return Task.ERROR

        return Task.Success
