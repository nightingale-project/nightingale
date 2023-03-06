#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_msgs.msg import RoomRunnerAction, RoomRunnerGoal


class NavigateTask(Task):
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
        tries = 3
        small_distance = 0.5
        for i in range(tries):
            self.action_client.send_goal(goal)
            self.action_client.wait_for_result()
            if self.action_client.get_result().final_distance_to_goal < small_distance:
                return TaskCodes.SUCCESS
            rospy.logerr(
                f"RoomRunner failed. The final distance {self.action_client.get_result().final_distance_to_goal} is high. Trying {tries-i-1} more times."
            )
            rospy.logerr(f"Give the robot more space")
            rospy.sleep(5)
        return TaskCodes.ERROR
