#!/usr/bin/env python

import rospy
import actionlib
import time

from actionlib_msgs.msg import GoalStatus
from nightingale_msgs.msg import TaskAction


class TaskActionServer:
    def __init__(self):
        rospy.init_node("task_action_server_node")

        self.server = actionlib.SimpleActionServer("task", TaskAction, self.goal_cb, False)
        self.server.start()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Bed. idle -> Nav -> Stock -> Bed. deliver -> Nav -> Idle

        self.server.set_succeeded()


def main():
    task_action_server = TaskActionServer()

    rospy.spin()


if __name__ == "__main__":
    main()