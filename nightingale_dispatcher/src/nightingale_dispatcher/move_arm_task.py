#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_manipulation.manipulation_action_client import ManipulationControl


class MoveArmTask(Task):
    def __init__(self):
        self.manipulation = ManipulationControl()
        if not self.manipulation.home_left() or not self.manipulation.home_right():
            rospy.logfatal("MoveArmTask: Failed to home arms. This is unrecoverable")

    def extend_restock(self):
        result = self.manipulation.extend_restock()
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR

    def retract_right_arm(self):
        result = self.manipulation.home_right()
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR

    def extend_handoff(self, point):
        result = self.manipulation.extend_handoff(point)
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR
