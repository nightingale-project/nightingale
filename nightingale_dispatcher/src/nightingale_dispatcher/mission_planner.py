#!/usr/bin/env python

import rospy
import actionlib
import queue

from actionlib_msgs.msg import GoalStatus
from nightingale_msgs.msg import MissionPlanAction
from handoff_task import HandoffTask
from idle_task import IdleTask
from navigate_task import NavigateTask
from stock_task import StockTask
from triage_task import TriageTask

class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Idle

        # Extract room number and bed number from goal (bell)
        status = NavigateTask(goal.room, "bed").execute()
        if status < 0:
            raise NotImplementedError()

        # Arrived at patient's bedside
        status = TriageTask().execute()
        if status < 0:
            raise NotImplementedError()

        if status == 0:
            # User didn't want anything
            raise NotImplementedError()
        else:
            # User wants some items

            # Go to stock room
            status = NavigateTask("stock").execute()
            if status < 0:
                raise NotImplementedError()

            # Arrived at stock area
            status = StockTask().execute()
            if status < 0:
                raise NotImplementedError()

            # Got items, go back to patient room
            status = NavigateTask(goal.room, "bed")
            if status < 0:
                raise NotImplementedError()

            # Arrived at patient's bedside
            status = HandoffTask().execute()
            if status < 0:
                raise NotImplementedError()
            

def main():
    mission_planner = MissionPlanner()

    rospy.spin()


if __name__ == "__main__":
    main()
