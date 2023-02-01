#!/usr/bin/env python3

import rospy

from nightingale_dispatcher.task_with_interface_comms import TaskWithInterfaceComms
import actionlib
from actionlib_msgs.msg import GoalStatus
from nightingale_msgs.msg import RoomRunnerAction, RoomRunnerGoal


class NavigateTask(TaskWithInterfaceComms):
    def __init__(self):
        action_name = "/nightingale_navigation/room_runner"
        self.action_client = actionlib.SimpleActionClient(action_name, RoomRunnerAction)
        rospy.loginfo(f"Waiting for {action_name} action server")
        if self.action_client.wait_for_server():
            rospy.loginfo(
                f"Found the {action_name} action server",
            )
        else:
            rospy.logfatal(
                f"Failed to find the {action_name} action server",
            )

    def execute(self, room_number, sublocation):
        goal = RoomRunnerGoal()
        goal.room_number = room_number
        goal.sublocation = sublocation
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return (
            Task.SUCCESS
            if self.action_client.get_state() == GoalStatus.SUCCEEDED
            else Task.ERROR
        )
