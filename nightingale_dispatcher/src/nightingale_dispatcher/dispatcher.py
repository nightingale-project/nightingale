#!/usr/bin/env python3

import queue
import rospy
import actionlib

from actionlib.simple_action_client import SimpleGoalState
from nightingale_msgs.msg import MissionPlanAction, MissionPlanGoal
from nightingale_msgs.srv import AddMissionPlan, AddMissionPlanResponse


class DispatcherNode:
    def __init__(self):
        rospy.init_node("dispatcher_node")

        self.mission_planner_client = actionlib.SimpleActionClient(
            "mission_planner", MissionPlanAction
        )
        rospy.loginfo("Dispatcher: waiting for Mission Planner")
        self.mission_planner_client.wait_for_server()

        self.missions = queue.Queue()

        self.add_mission_service = rospy.Service(
            "/nightingale/add_mission", AddMissionPlan, self.handle_add_mission
        )

    def handle_add_mission(self, req):
        goal = MissionPlanGoal()
        goal.name = req.name
        goal.phase = req.phase

        self.missions.put(goal)
        if self.mission_planner_client.simple_state == SimpleGoalState.DONE:
            # No goal in progress, execute goal now
            self.mission_planner_client.send_goal(
                self.missions.get(), self.handle_done_task
            )
        ret = AddMissionPlanResponse()
        ret.status = True
        return ret

    def handle_done_task(self, status, result):
        if not self.missions.empty():
            self.mission_planner_client.send_goal(
                self.missions.get(), self.handle_done_task
            )


def main():
    dispatcher = DispatcherNode()

    rospy.spin()


if __name__ == "__main__":
    main()
