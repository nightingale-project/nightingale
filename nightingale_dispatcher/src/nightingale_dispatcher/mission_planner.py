#!/usr/bin/env python

import rospy
import actionlib
import time

from actionlib_msgs.msg import GoalStatus
from nightingale_msgs.msg import MissionPlanAction


class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Bed. idle -> Nav -> Stock -> Bed. deliver -> Nav -> Idle

        raise NotImplementedError()


def main():
    mission_planner = MissionPlanner()

    rospy.spin()


if __name__ == "__main__":
    main()
