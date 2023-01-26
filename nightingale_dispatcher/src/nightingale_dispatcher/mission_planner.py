#!/usr/bin/env python

import rospy
import actionlib
import queue

from actionlib_msgs.msg import GoalStatus
from nightingale_msgs.msg import MissionPlanAction
from nightingale_dispatcher.handoff_task import HandoffTask
from nightingale_dispatcher.idle_task import IdleTask
from nightingale_dispatcher.navigate_task import NavigateTask
from nightingale_dispatcher.stock_task import StockTask
from nightingale_dispatcher.task import Task
from nightingale_dispatcher.triage_task import TriageTask

class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

        self.handoff_task = HandoffTask()
        self.idle_task = IdleTask()
        self.navigate_task = NavigateTask()
        self.stock_task = StockTask()
        self.triage_task = TriageTask()

        self.states = queue.Queue()
    
    def go_to_patient(self):
        # Extract room number and bed number from goal (bell)
        status = self.navigate_task.execute(self.room, "bed")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.states.put(self.triage_patient)

    def triage_patient(self):
        # Arrived at patient's bedside
        status = self.triage_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()

        if status == TriageTask.TIMEOUT:
            # User didn't want anything
            self.states.put(self.go_idle)
        else:
            # User wants some items
            self.states.put(self.go_to_stock)

    def go_to_stock(self):
        # Go to stock room
        status = self.navigate_task.execute("stock")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.states.put(self.get_items)

    def get_items(self):
        # Arrived at stock area
        status = self.stock_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()
        self.states.put(self.return_to_patient)

    def return_to_patient(self):
        # Got items, go back to patient room
        status = self.navigate_task.execute(self.room, "bed")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.states.put(self.handoff_items)

    def handoff_items(self):
        # Arrived at patient's bedside
        status = self.handoff_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()
        self.states.put(self.triage_patient)

    def go_idle(self):
        status = self.idle_task.execute()
        if status == Task.ERROR:
            raise NotImplementedError()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Idle

        self.room = goal.room
        self.states.put(self.go_to_patient)

        while not self.states.empty():
            state = self.states.get()
            state()
        
        self.server.set_succeeded()
            
def main():
    mission_planner = MissionPlanner()

    rospy.spin()


if __name__ == "__main__":
    main()
