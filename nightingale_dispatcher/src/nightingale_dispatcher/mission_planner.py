#!/usr/bin/env python3

import rospy
import actionlib
import queue
from enum import Enum

from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from nightingale_msgs.msg import MissionPlanAction
from nightingale_dispatcher.navigate_task import NavigateTask
from nightingale_dispatcher.move_arm_task import MoveArmTask
from nightingale_dispatcher.send_interface_request_task import SendInterfaceRequestTask
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig, RobotStatus


# enum for phase status
class PhaseStatus(Enum):
    PHASE_COMPLETE = 1
    PHASE_FAIL = 0


class MissionPlanner(PhaseStatus):
    def __init__(self):
        rospy.init_node("mission_planner_node")

        self.estop_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.estop_cb
        )
        self.cfg = rospy.get_param("/nightingale_utils/joint_configurations")
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

        # update to driving screen
        self.send_interface_request_task.execute(RobotStatus.DRIVING)

        status = self.navigate_task.execute(self.room, "bedside")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.triage_patient_phase)
        return PhaseStatus.PHASE_COMPLETE

    def go_home_base_phase(self):
        rospy.loginfo("Nightingale Mission Planner going home")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added

        # update to driving screen
        self.send_interface_request_task.execute(RobotStatus.DRIVING)

        status = self.navigate_task.execute("home", "default")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.go_idle_phase)
        return PhaseStatus.PHASE_COMPLETE

    def triage_patient_phase(self):
        # Arrived at patient's bedside
        # get patient input
        rospy.loginfo("Nightingale Mission Planner waiting on user input")
        task_response = self.send_interface_request_task.execute(
            RobotStatus.BEDSIDE_IDLE
        )

        if task_response == TaskCodes.ERROR:
            raise NotImplementedError()
        if task_response == TaskCodes.WD_TIMEOUT or task_response == TaskCodes.DISMISS:
            # User didn't want anything or timedout
            self.phases.put(self.go_home_base_phase)
        elif task_response == TaskCodes.STOCK_ITEMS:
            # User wants some items
            self.phases.put(self.go_to_stock_phase)
        else:
            # should never reach here
            rospy.loginfo(f"Unknown input {task_reponse}")
            raise NotImplementedError()
        return PhaseStatus.PHASE_COMPLETE

    def go_to_stock_phase(self):
        rospy.loginfo("Nightingale Mission Planner going to stock")

        # update to driving screen
        self.send_interface_request_task.execute(RobotStatus.DRIVING)

        status = self.navigate_task.execute("stock", "default")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.get_items_phase)
        return PhaseStatus.PHASE_COMPLETE

    def get_items_phase(self):
        rospy.loginfo("Nightingale Mission Planner getting items")
        # Arrived at stock area

        # arm extend stuff
        joint_values = self.cfg["right_arm_extended_handoff"]["joints"]
        status = self.move_arm_task.execute(joint_values)
        # get nurse input
        task_reponse = self.send_interface_request_task.execute(
            RobotStatus.ITEM_STOCK_REACHED
        )

        # arm retract stuff
        joint_values = self.cfg["right_arm_home"]["joints"]
        status = self.move_arm_task.execute(joint_values)

        if task_reponse == TaskCodes.ERROR:
            raise NotImplementedError()
        elif task_reponse == TaskCodes.DELIVER_ITEMS:
            self.phases.put(self.return_to_patient_phase)
        elif task_reponse == TaskCodes.DISMISS:
            # nurse cancelled
            self.phases.put(self.go_home_base_phase)
        else:
            # should not get here but add so something happens
            rospy.loginfo(f"Unknown input by nurse {task_reponse}")
            self.phases.put(self.go_home_base_phase)
        return PhaseStatus.PHASE_COMPLETE

    def return_to_patient_phase(self):
        rospy.loginfo("Nightingale Mission Planner returning to patient")
        # Got items, go back to patient room
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added

        # update to driving screen
        self.send_interface_request_task.execute(RobotStatus.DRIVING)

        status = self.navigate_task.execute(self.room, "bedside")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.handoff_items_phase)
        return PhaseStatus.PHASE_COMPLETE

    def handoff_items_phase(self):
        rospy.loginfo("Nightingale Mission Planner starting to hand items")
        # Arrived at patient's bedside

        # show arm movement and get input to start
        task_response = self.send_interface_request_task.execute(
            RobotStatus.BEDSIDE_DELIVER
        )

        # extend arm
        joint_values = self.cfg["right_arm_extended_handoff"]["joints"]
        status = self.move_arm_task.execute(joint_values)

        # arm extended
        task_response = self.send_interface_request_task.execute(
            RobotStatus.ARM_EXTENDED
        )

        # show arm movement and get input to start
        # status = self.send_interface_request_task.execute(RobotStatus.RETRACTING_ARM)

        # retract arm
        joint_values = self.cfg["right_arm_home"]["joints"]
        status = self.move_arm_task.execute(joint_values)

        # send to screen arm retracted
        task_response = self.send_interface_request_task.execute(
            RobotStatus.ARM_RETRACTED
        )

        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        # when done automatically goes back to triage patient
        self.phases.put(self.triage_patient_phase)
        return PhaseStatus.PHASE_COMPLETE

    def go_idle_phase(self):
        # cleanup and exit
        # Update idle screen
        status = self.send_interface_request_task.execute(RobotStatus.IDLE_HOME)

        # move arms to home position?

        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        return PhaseStatus.PHASE_COMPLETE

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Home -> Idle

        self.room = goal.name
        self.phases.put(self.go_to_patient_phase)

        while not self.phases.empty():
            phase = self.phases.get()
            status = phase()

            rospy.loginfo(status)
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
