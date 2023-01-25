#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import actionlib

class HumanPoseEstimation:
    def __init__(self):
        rospy.init_node("human_pose_estimation_server", anonymous=True)
        self.human_pose_estimation_server = action.SimpleActionServer('human_pose_estimation', HumanPoseAction, self.execute, False)
        self.human_pose_estimation_server.start()


        def execute(self, goal):
            #face detection stuff
            self.server.set_succeeded()


if __name__ == '__main':
    rospy.init_node("human_pose_estimation_server", anonymous=True)
    server = HumanPoseEstimation()
    rospy.spin()

