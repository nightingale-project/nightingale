# /usr/bin/env python3
import queue
from enum import Enum

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from std_srvs.srv import Empty
from nightingale_msgs.msg import MissionPlanAction, MissionPlanGoal
from geometry_msgs.msg import Point
from nightingale_dispatcher.navigate_task import NavigateTask
from nightingale_dispatcher.move_arm_task import MoveArmTask
from nightingale_dispatcher.move_body_task import MoveBodyTask
from nightingale_dispatcher.send_interface_request_task import SendInterfaceRequestTask
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_ros_bridge.bridge_interface_config import BridgeConfig, RobotStatus
from nightingale_dispatcher.estimate_pose_task import EstimatePoseTask


# enum for phase status
class PhaseStatus(Enum):
    PHASE_COMPLETE = 1
    PHASE_FAIL = 0


class MissionPlanner:
    def __init__(self):
        rospy.init_node("mission_planner_node")

        rospy.loginfo("Waiting to get octomap service")
        rospy.wait_for_service("/clear_octomap")
        rospy.loginfo("Got octomap service")
        self.clear_octomap = rospy.ServiceProxy("/clear_octomap", Empty)
        rospy.loginfo("Clearing octomap")
        self.clear_octomap()
        rospy.loginfo("Cleared octomap")

        self.estop_sub = rospy.Subscriber(
            BridgeConfig.USER_INPUT_TOPIC, String, self.estop_cb
        )
        self.navigate_task = NavigateTask()
        self.move_arm_task = MoveArmTask()
        self.move_body_task = MoveBodyTask()
        self.estimate_pose_task = EstimatePoseTask()
        self.send_interface_request_task = SendInterfaceRequestTask()

        self.phases = queue.Queue()
        self.phase_map = {
            MissionPlanGoal.GO_TO_PATIENT_PHASE: self.go_to_patient_phase,
            MissionPlanGoal.GO_HOME_BASE_PHASE: self.go_home_base_phase,
            MissionPlanGoal.TRIAGE_PATIENT_PHASE: self.triage_patient_phase,
            MissionPlanGoal.GO_TO_STOCK_PHASE: self.go_to_stock_phase,
            MissionPlanGoal.GET_ITEMS_PHASE: self.get_items_phase,
            MissionPlanGoal.RETURN_TO_PATIENT_PHASE: self.return_to_patient_phase,
            MissionPlanGoal.HANDOFF_ITEMS_PHASE: self.handoff_items_phase,
            MissionPlanGoal.GO_IDLE_PHASE: self.go_idle_phase,
        }

        self.server = actionlib.SimpleActionServer(
            "mission_planner", MissionPlanAction, self.goal_cb, False
        )
        self.server.start()

    def go_to_patient_phase(self):
        rospy.loginfo("Nightingale Mission Planner going to patient")
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added

        # update to driving screen
        task_response = self.send_interface_request_task.execute(RobotStatus.DRIVING)

        rospy.loginfo("Lowering body to home")
        status = self.move_body_task.home()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO LOWER TORSO")
            raise NotImplementedError()

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
        task_response = self.send_interface_request_task.execute(RobotStatus.DRIVING)

        rospy.loginfo("Lowering body to home")
        status = self.move_body_task.home()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO LOWER TORSO")
            raise NotImplementedError()

        status = self.navigate_task.execute("home", "default")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.go_idle_phase)
        return PhaseStatus.PHASE_COMPLETE

    def triage_patient_phase(self):
        # Arrived at patient's bedside

        self.clear_octomap()

        rospy.loginfo("Raising body to handoff")
        status = self.move_body_task.handoff()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO RAISE TORSO")
            raise NotImplementedError()

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
            rospy.loginfo(f"Unknown input {task_response}")
            raise NotImplementedError()
        return PhaseStatus.PHASE_COMPLETE

    def go_to_stock_phase(self):
        rospy.loginfo("Nightingale Mission Planner going to stock")

        # update to driving screen
        task_response = self.send_interface_request_task.execute(RobotStatus.DRIVING)

        rospy.loginfo("Lowering body to home")
        status = self.move_body_task.home()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO LOWER TORSO")
            raise NotImplementedError()

        status = self.navigate_task.execute("stock", "default")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.get_items_phase)
        return PhaseStatus.PHASE_COMPLETE

    def get_items_phase(self):
        rospy.loginfo("Nightingale Mission Planner getting items")
        # Arrived at stock area
        self.clear_octomap()

        # arm extend stuff
        rospy.loginfo("Nightingale Mission Planner extending arm for stocking")
        if self.move_arm_task.extend_restock() != TaskCodes.SUCCESS:
            rospy.logerr("Nightingale Mission Planner failed to extend arm for handoff")
            raise NotImplementedError()
        # get nurse input
        task_response = self.send_interface_request_task.execute(
            RobotStatus.ITEM_STOCK_REACHED
        )

        # arm retract stuff
        rospy.loginfo("Nightingale Mission Planner retracting arm after stocking")
        if self.move_arm_task.retract_right_arm() != TaskCodes.SUCCESS:
            rospy.logerr("Nightingale Mission Planner failed to retract arm")
            raise NotImplementedError()
        rospy.loginfo("Nightingale Mission Planner retracted arm")

        # add a block to check arm status first before interpreting input

        if task_response == TaskCodes.ERROR:
            raise NotImplementedError()
        elif task_response == TaskCodes.DELIVER_ITEMS:
            self.phases.put(self.return_to_patient_phase)
        elif task_response == TaskCodes.DISMISS:
            # nurse cancelled
            self.phases.put(self.go_home_base_phase)
        else:
            # should not get here but add so something happens
            rospy.loginfo(f"Unknown input by nurse {task_response}")
            self.phases.put(self.go_home_base_phase)
        return PhaseStatus.PHASE_COMPLETE

    def return_to_patient_phase(self):
        rospy.loginfo("Nightingale Mission Planner returning to patient")
        # Got items, go back to patient room
        # Assume door is open
        # TODO: first go to doorside then bedside when door opening added

        # update to driving screen
        task_reponse = self.send_interface_request_task.execute(RobotStatus.DRIVING)

        rospy.loginfo("Lowering body to home")
        status = self.move_body_task.home()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO LOWER TORSO")
            raise NotImplementedError()

        status = self.navigate_task.execute(self.room, "bedside")
        if status == TaskCodes.ERROR:
            raise NotImplementedError()
        self.phases.put(self.handoff_items_phase)
        return PhaseStatus.PHASE_COMPLETE

    def handoff_items_phase(self):
        rospy.loginfo("Nightingale Mission Planner starting to hand items")
        # Arrived at patient's bedside
        self.clear_octomap()

        rospy.loginfo("Raising body to handoff")
        status = self.move_body_task.handoff()
        if status != TaskCodes.SUCCESS:
            rospy.logerr("UNABLE TO RAISE TORSO")
            raise NotImplementedError()

        rospy.sleep(2.5)

        # pose estimation
        # status, pose_result = self.estimate_pose_task.execute("body")
        # bin_goal_pt = pose_result.bin_goal.point
        # if not self.move_arm_task.witihin_workspace(bin_goal_pt):
        #    rospy.logerr(f"Nightingale Perception returned a point not within the workspace: {bin_goal_pt}. Trying closer point")
        #    alpha = 0.9
        #    for _ in range(3):
        #        bin_goal_pt.x = bin_goal_pt.x * alpha
        #        bin_goal_pt.y = bin_goal_pt.y * alpha
        #        bin_goal_pt.z = bin_goal_pt.z * alpha
        #        if self.move_arm_task.witihin_workspace(bin_goal_pt): break
        #        rospy.logerr(f"Closer point still outside of workspace, trying even closer point {bin_goal_pt}")
        # rospy.loginfo(f"node returns {pose_result}")
        status, pose_result = self.estimate_pose_task.execute("body")
        bin_goal_pt = pose_result.bin_goal.point

        rospy.loginfo(f"node returns {pose_result}")

        task_response = self.send_interface_request_task.execute(
            RobotStatus.BEDSIDE_DELIVER
        )

        # extend arm
        rospy.loginfo("Nightingale Mission Planner extending arm for handoff")

        if status != TaskCodes.SUCCESS or not self.move_arm_task.within_workspace(
            bin_goal_pt
        ):
            rospy.logwarn("UNABLE TO FIND POSE. FALLING BACK TO SAFE HANDOFF POSITION")
            status = self.move_arm_task.extend_restock()
        else:
            status = self.move_arm_task.extend_handoff(bin_goal_pt)

        if status != TaskCodes.SUCCESS:
            rospy.logerr("Nightingale Mission Planner failed to extend arm for handoff")
            raise NotImplementedError()
        rospy.loginfo("Nightingale Mission Planner extended arm for handoff")

        # arm extended
        task_response = self.send_interface_request_task.execute(
            RobotStatus.ARM_EXTENDED
        )

        # show arm movement and get input to start
        status = self.send_interface_request_task.execute(RobotStatus.RETRACTING_ARM)

        # retract arm
        rospy.loginfo("Nightingale Mission Planner retracting arm after handoff")
        if self.move_arm_task.retract_right_arm() != TaskCodes.SUCCESS:
            rospy.logerr("Nightingale Mission Planner failed to retract arm")
            raise NotImplementedError()
        rospy.loginfo("Nightingale Mission Planner retracted arm after handoff")

        # when done automatically goes back to triage patient
        self.phases.put(self.triage_patient_phase)
        return PhaseStatus.PHASE_COMPLETE

    def go_idle_phase(self):
        # cleanup and exit
        rospy.loginfo("Nightingale Mission Planner idling")

        # Update idle screen
        task_response = self.send_interface_request_task.execute(RobotStatus.IDLE_HOME)

        if task_response == TaskCodes.ERROR:
            raise NotImplementedError()
        return PhaseStatus.PHASE_COMPLETE

    def goal_cb(self, goal):
        # TODO execute subtasks in order and report status
        # Nav -> Triage -> Nav -> Stock -> Nav ->
        #   Handoff -> Home -> Idle

        self.room = goal.name
        try:
            self.phases.put(self.phase_map[goal.phase])
        except KeyError:
            rospy.logerr(
                f"Nightingale Mission Planner: Invalid phase {goal.phase} received"
            )
            self.server.set_aborted()
            return

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
