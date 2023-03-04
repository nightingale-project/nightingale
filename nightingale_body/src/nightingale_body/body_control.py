#!/usr/bin/env python3

import actionlib
import numpy as np
import rospy

from moveit_action_handlers.msg import (
    MoveToJointsMoveItAction,
    MoveToJointsMoveItGoal,
    PropertyValuePair,
)
from nightingale_msgs.srv import (
    RobotConfigurationLookup,
    RobotConfigurationLookupRequest,
)
from sensor_msgs.msg import JointState


class BodyJointControl:
    CLOSE_DIST_TOL = 1e-2  # m
    CLOSE_ANGLE_TOL = 0.087  # rad = 5 deg

    def __init__(self):
        self.head_ac = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/head/joint_ctrl", MoveToJointsMoveItAction
        )
        rospy.loginfo("Waiting for head joint moveit action server")
        self.head_ac.wait_for_server()
        rospy.loginfo("Found head joint moveit action server")

        self.torso_ac = actionlib.simple_action_client.SimpleActionClient(
            "/moveit_action_handlers/torso/joint_ctrl", MoveToJointsMoveItAction
        )
        rospy.loginfo("Waiting for torso joint moveit action server")
        self.torso_ac.wait_for_server()
        rospy.loginfo("Found torso joint moveit action server")

    def _make_goal(self, jnt_name, jnt_pos, ee_vel=0.5, ee_accel=0.5, timeout=5):
        goal = MoveToJointsMoveItGoal()

        goal.endEffectorVelocity = ee_vel
        goal.endEffectorAcceleration = ee_accel
        goal.timeoutSeconds = timeout

        for name, pos in zip(jnt_name, jnt_pos):
            joint_pair = PropertyValuePair()
            joint_pair.name = name
            joint_pair.value = pos
            goal.joint_pairs.append(joint_pair)
        return goal

    def cmd_head(self, target_jnt_name, target_jnt_pos):
        if self.head_is_close(target_jnt_name, target_jnt_pos):
            return True

        goal = self._make_goal(target_jnt_name, target_jnt_pos)

        self.head_ac.send_goal(goal)
        return self.head_ac.wait_for_result()

    def cmd_torso(self, target_jnt_name, target_jnt_pos):
        if self.torso_is_close(target_jnt_name, target_jnt_pos):
            return True

        goal = self._make_goal(target_jnt_name, target_jnt_pos)

        self.torso_ac.send_goal(goal)
        return self.torso_ac.wait_for_result()

    def head_is_close(self, target_jnt_name, target_jnt_pos):
        curr_jnt_state = rospy.wait_for_message("/movo/head/joint_states", JointState)
        curr_jnt_pos = {
            name: pos for name, pos in zip(curr_jnt_state.name, curr_jnt_state.pos)
        }

        diff = np.zeros(len(target_jnt_name))
        for idx, (tgt_name, tgt_pos) in enumerate(zip(target_jnt_name, target_jnt_pos)):
            try:
                diff[idx] = np.abs(curr_jnt_pos[tgt_name] - tgt_pos)
            except KeyError:
                continue

        return np.all(diff < self.CLOSE_ANGLE_TOL)

    def torso_is_close(self, target_jnt_name, target_jnt_pos):
        curr_jnt_state = rospy.wait_for_message(
            "/movo/linear_actuator/joint_states", JointState
        )
        curr_jnt_pos = {
            name: pos for name, pos in zip(curr_jnt_state.name, curr_jnt_state.pos)
        }

        diff = np.zeros(len(target_jnt_name))
        for idx, (tgt_name, tgt_pos) in enumerate(zip(target_jnt_name, target_jnt_pos)):
            try:
                diff[idx] = np.abs(curr_jnt_pos[tgt_name] - tgt_pos)
            except KeyError:
                continue

        return np.all(diff < self.CLOSE_DIST_TOL)


class BodyControl:
    def __init__(self):
        robot_config_service = "/nightingale/robot_configuration_lookup"
        rospy.loginfo(f"Manipulation Control: waiting for robot configuration service")
        rospy.wait_for_service(robot_config_service)
        lookup_client = rospy.ServiceProxy(
            robot_config_service, RobotConfigurationLookup
        )
        rospy.loginfo(
            "Nightingale Manipulation Control found robot configuration lookup service server"
        )
        try:
            response = lookup_client("home", RobotConfigurationLookupRequest.HEAD)
            self.head_joint_names = response.jnt_states.names
            self.head_home_joint_values = response.jnt_states.position

            response = lookup_client("home", RobotConfigurationLookupRequest.TORSO)
            self.torso_joint_names = response.jnt_states.names
            self.torso_home_joint_values = response.jnt_states.position

            response = lookup_client("handoff", RobotConfigurationLookupRequest.TORSO)
            self.torso_joint_names = response.jnt_states.names
            self.torso_handoff_joint_values = response.jnt_states.position

        except rospy.ServiceException as e:
            rospy.logerr(
                f"Nightingale Manipulation Control failed to call robot configuration lookup service {e}"
            )

        self.body_jnt_ctrl = BodyJointControl()

    def home(self):
        self.body_jnt_ctrl.cmd_head(self.head_joint_names, self.head_home_joint_values)
        self.body_jnt_ctrl.cmd_torso(
            self.torso_joint_names, self.torso_home_joint_values
        )

    def handoff(self):
        self.body_jnt_ctrl.cmd_torso(
            self.torso_joint_names, self.torso_handoff_joint_values
        )


if __name__ == "__main__":
    rospy.init_node("body_control_test")
    body = BodyControl()

    rospy.loginfo("Going to home")
    body.home()
    rospy.loginfo("Arrived at home")
    rospy.sleep(rospy.Duration(2))

    rospy.loginfo("Going to handoff")
    body.handoff()
    rospy.loginfo("Arrived at handoff")
