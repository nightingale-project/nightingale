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
        result = None
        for _ in range(0, 3):
            self.action_client.send_goal(goal)
            self.action_client.wait_for_result()
            result = self.action_client.get_result()
            # allow retrial of the estimation in case cannot find pose
            if self.action_client.get_state() == GoalStatus.SUCCEEDED:
                return (TaskCodes.SUCCESS, result)

        return (TaskCodes.ERROR, result)
