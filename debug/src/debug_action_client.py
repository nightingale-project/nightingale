#! /usr/bin/env python
import rospy
# Brings in the SimpleActionClient
import actionlib
import time


# control_msgs/FollowJointTrajectoryActionFeedback
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def moveit_tojoints_client():
    # ARM
    arm_client = actionlib.simple_action_client.SimpleActionClient(
        '/movo/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    print("waiting for server...")
    arm_client.wait_for_server()
    print("connected to server")
    # Creates a goal to send to the action server arm.

    goal = FollowJointTrajectoryGoal()
    point = JointTrajectoryPoint()
    point.positions = [1.8055517131984544, 0.7156651544664865, 0.4545099853350272, 1.2894898166431348,
                       -3.012074075031533, 0.9220841073974382, -2.4957912692637914]
    point.time_from_start.secs = 3
    goal.trajectory.points.append(point)

    goal.trajectory.joint_names = ["right_shoulder_pan_joint",
                                   "right_shoulder_lift_joint",
                                   "right_arm_half_joint",
                                   "right_elbow_joint",
                                   "right_wrist_spherical_1_joint",
                                   "right_wrist_spherical_2_joint",
                                   "right_wrist_3_joint"]

    new_goal = FollowJointTrajectoryGoal()
    new_point = JointTrajectoryPoint()
    new_point.positions = [2.2899584780887663, 0.6968041149943274, -0.4708056683081403, 1.4458962021990525,
                           -2.523168565269806, 0.9773449663326914, -2.494001358695149]
    new_point.time_from_start.secs = 3
    new_goal.trajectory.points.append(new_point)

    new_goal.trajectory.joint_names = ["right_shoulder_pan_joint",
                                          "right_shoulder_lift_joint",
                                          "right_arm_half_joint",
                                          "right_elbow_joint",
                                          "right_wrist_spherical_1_joint",
                                          "right_wrist_spherical_2_joint",
                                          "right_wrist_3_joint"]

    # send goal
    arm_client.send_goal(goal)
    print("goal sent")
    # Waits for the server to finish performing the action.
    arm_client.wait_for_result()
    print(str(arm_client.get_result()))

    time.sleep(2)

    arm_client.send_goal(new_goal)
    arm_client.wait_for_result()
    print(str(arm_client.get_result()))


if __name__ == '__main__':
    rospy.init_node('test_joints_client_python_node')
    result = moveit_tojoints_client()
