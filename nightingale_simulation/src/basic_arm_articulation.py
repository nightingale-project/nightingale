#!/usr/bin/env python3
'''
This is a dummy test script for moving the arms in simulation
The motion is random. THIS SHOULD NOT BE RUN ON REAL HW
'''
import rospy
import sys
import actionlib
import numpy as np
import actionlib_tutorials.msg
from control_msgs.msg import FollowJointTrajectoryAction,FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def articulate():
    joint_names = ['left_shoulder_pan_joint','left_shoulder_lift_joint','left_arm_half_joint','left_elbow_joint','left_wrist_spherical_1_joint','left_wrist_spherical_2_joint','left_wrist_3_joint']
    client = actionlib.SimpleActionClient('/movo/left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    cur_pos = [0.0 for _ in range(len(joint_names))]
    while not rospy.is_shutdown():
        l,u = np.radians([-10,10])
        next_pos = np.random.uniform(low=l, high=u, size=(len(joint_names),)).tolist()
        
        # Move the arm from cur to next
        print(f'Moving arm to {next_pos}')
        msg = FollowJointTrajectoryAction()
        goal = msg.action_goal
        goal.header.stamp = rospy.Time.now()+rospy.Duration(0.5)
        goal.goal.trajectory.joint_names = joint_names
        point0 = JointTrajectoryPoint()
        point0.positions = cur_pos
        point0.velocities = [0.0 for _ in range(len(joint_names))]
        point1 = JointTrajectoryPoint()
        point1.positions = next_pos
        point1.velocities = [0.0 for _ in range(len(joint_names))]
        point1.time_from_start = rospy.Duration(2)
        goal.goal.trajectory.points = [point0,point1]
        client.send_goal(msg.action_goal.goal)
        client.wait_for_result()
        cur_pos = next_pos

        print(client.get_result())
        assert client.get_result().error_code == 0

        rospy.sleep(3)

    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('articulate')
        articulate()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
