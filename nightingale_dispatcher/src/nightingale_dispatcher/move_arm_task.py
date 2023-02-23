#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task
import actionlib
from actionlib_msgs.msg import GoalStatus
from nightingale_manipulation.manipulation_action_client import ManipulationControl


class MoveArmTask(Task):
    def __init__(self):
        self.manipulation = ManipulationControl()
        if not self.manipulation.home():
            rospy.logfatal("MoveArmTask: Failed to home arms. This is unrecoverable")

    def extend_restock(self):
        result = self.manipulation.extend_restock()
        assert type(result) is bool
        return Task.SUCCESS if result else Task.ERROR

    def retract_right_arm(self):
        result = self.manipulation.home()
        assert type(result) is bool
        return Task.SUCCESS if result else Task.ERROR

    def extend_handoff(self, point):
        result = self.manipulation.extend_handoff(point)
        assert type(result) is bool
        return Task.SUCCESS if result else Task.ERROR
