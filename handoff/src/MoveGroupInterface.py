from __future__ import print_function
from six.moves import input

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, tau, dist, fabs, cos

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class JacoMoveitInterface(object):

    def __init__(self, move_group):
        super(JacoMoveitInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = move_group
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.group_names)
        self.i = 0

    def go_to_joint_state(self, joint_goal):

        assert type(joint_goal) is type(self.move_group.get_current_joint_values())

        success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()

        return success

    def increment_joint_state(self):
        joint_goal = self.move_group.get_current_joint_values()
        self.i += 1
        self.i = self.i % 7
        joint_goal[self.i] += 0.1

        success = self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        return success

    def go_to_pose_goal(self):
        # code from tutorial, do not use
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        move_group.set_pose_target(pose_goal)

        success = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return success

    def plan_cartesian_path(self, scale=1):
        # code from tutorial, do not use
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        return plan, fraction

    def execute_plan(self, plan):
        self.move_group.execute(plan, wait=True)
