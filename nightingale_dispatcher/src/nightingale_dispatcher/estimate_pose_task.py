#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_msgs.msg import PoseEstimationAction, PoseEstimationGoal


class EstimatePoseTask(Task):
    def __init__(self):
        action_name = "/nightingale_perception/pose_estimation"
        self.action_client = actionlib.SimpleActionClient(
            action_name, PoseEstimationAction
        )
        rospy.loginfo(f"Waiting for {action_name} action server")
        if self.action_client.wait_for_server():
            rospy.loginfo(
                f"Found the {action_name} action server",
            )
        else:
            rospy.logfatal(
                f"Failed to find the {action_name} action server",
            )

    def execute(self, target):
        goal = PoseEstimationGoal()
        goal.target = target
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        bin_goal = self.action_client.get_result()
        return (
            TaskCodes.SUCCESS
            if self.action_client.get_state() == GoalStatus.SUCCEEDED
            else TaskCodes.ERROR,
            bin_goal,
        )
