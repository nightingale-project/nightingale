#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task
import actionlib
from actionlib_msgs.msg import GoalStatus
from nightingale_manipulation.manipulation_action_client import ManipulationControl


class MoveArmTask(Task):
    def __init__(self):
        self.manipulation = ManipulationControl()

    def execute(self, end_pose):
        assert len(end_pose) is 7
        self.manipulation.jnt_ctrl.cmd_right_arm(end_pose)
        return Task.SUCCESS
