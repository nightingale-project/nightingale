#!/usr/bin/env python3

import rospy
import actionlib
import queue

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from nightingale_msgs.msg import MissionPlanAction
from nightingale_dispatcher.navigate_task import NavigateTask
from nightingale_dispatcher.move_arm_task import MoveArmTask
from nightingale_dispatcher.send_interface_request_task import SendInterfaceRequestTask
from nightingale_dispatcher.task import Task
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig


class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.estop_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.estop_cb
        )

        self.navigate_task = NavigateTask()
        self.move_arm_task = MoveArmTask()
        self.send_interface_request_task = SendInterfaceRequestTask()

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

        self.phases = queue.Queue()

    def go_to_patient_phase(self):
        rospy.loginfo("Nightingale Mission Planner going to patient")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute(self.room, "bedside")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.triage_patient_phase)

    def go_home_base_phase(self):
        rospy.loginfo("Nightingale Mission Planner going home")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute("home", "default")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.go_idle_phase)

    def triage_patient_phase(self):
        # Arrived at patient's bedside
        if status == Task.ERROR:
            raise NotImplementedError()
        if status == TriageTask.TIMEOUT:
            # User didn't want anything
            self.phases.put(self.go_home_base_phase)
        else:
            # User wants some items
            self.phases.put(self.go_to_stock_phase)

    def go_to_stock_phase(self):
        rospy.loginfo("Nightingale Mission Planner going to stock")
        status = self.navigate_task.execute("stock", "default")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.get_items_phase)

    def get_items_phase(self):
        # Arrived at stock area
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.return_to_patient_phase)

    def return_to_patient_phase(self):
        rospy.loginfo("Nightingale Mission Planner returning to patient")
        # Got items, go back to patient room
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added
        status = self.navigate_task.execute(self.room, "bedside")
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.handoff_items_phase)

    def handoff_items_phase(self):
        # Arrived at patient's bedside
        if status == Task.ERROR:
            raise NotImplementedError()
        self.phases.put(self.triage_patient_phase)

    def go_idle_phase(self):
        # cleanup and exit
        if status == Task.ERROR:
            raise NotImplementedError()

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Home -> Idle

        self.room = goal.name
        self.phases.put(self.go_to_patient_phase)

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
