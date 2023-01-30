#!/usr/bin/env python3

import rospy
import actionlib
import queue

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from nightingale_msgs.msg import MissionPlanAction
from nightingale_dispatcher.handoff_task import HandoffTask
from nightingale_dispatcher.idle_task import IdleTask
from nightingale_dispatcher.navigate_task import NavigateTask
from nightingale_dispatcher.stock_task import StockTask
from nightingale_dispatcher.task import Task
from nightingale_dispatcher.triage_task import TriageTask
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig


class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.estop_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.estop_cb
        )

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

        self.handoff_task = HandoffTask()
        self.idle_task = IdleTask()
        self.navigate_task = NavigateTask()
        self.stock_task = StockTask()
        self.triage_task = TriageTask()

        self.phases = queue.Queue()

    def go_to_patient(self):
        rospy.loginfo("Nightingale Mission Planner going to patient")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute(self.room, "bedside")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.triage_patient)

    def go_home(self):
        rospy.loginfo("Nightingale Mission Planner going home")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute("home", "")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.go_idle)

    def triage_patient(self):
        # Arrived at patient's bedside
        status = self.triage_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()

        if status == TriageTask.TIMEOUT:
            # User didn't want anything
            self.phases.put(self.go_idle)
        else:
            # User wants some items
            self.phases.put(self.go_to_stock)

    def go_to_stock(self):
        rospy.loginfo("Nightingale Mission Planner going to stock")
        status = self.navigate_task.execute("stock", "")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.get_items)

    def get_items(self):
        # Arrived at stock area
        status = self.stock_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.return_to_patient)

    def return_to_patient(self):
        rospy.loginfo("Nightingale Mission Planner returning to patient")
        # Got items, go back to patient room
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute(self.room, "bedside")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.handoff_items)

    def handoff_items(self):
        # Arrived at patient's bedside
        status = self.handoff_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.triage_patient)

    def go_idle(self):
        status = self.idle_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Home -> Idle

        self.room = goal.room
        self.phases.put(self.go_to_patient)

        while not self.phases.empty():
            phase = self.phases.get()
            status = phase()

            if not status:
                self.server.set_aborted()
                return

        self.server.set_succeeded()

    def estop_cb(self, msg):
        self.server.set_aborted()


def main():
    mission_planner = MissionPlanner()

    rospy.spin()


if __name__ == "__main__":
    main()
