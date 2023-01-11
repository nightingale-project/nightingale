#! /usr/bin/env python

import roslib
import rospy
import actionlib

from nightengale_msgs.msg import MoveToAction, MoveToFeedback, MoveToResult
from MoveGroupInterface import JacoMoveitInterface


class MoveToServer:
    feedback = MoveToFeedback()
    result = MoveToResult()

    def __init__(self):
        self.server = actionlib.SimpleActionServer('move_to', MoveToAction, self.execute, False)
        self.left_arm = JacoMoveitInterface("left_arm")
        self.right_arm = JacoMoveitInterface("right_arm")
        self.server.start()

    def execute(self, goal):
        if self.left_arm.increment_joint_state():
            rospy.loginfo("success")
        else:
            rospy.loginfo("failed")

        self.server.set_succeeded(self.result)


def main():
    rospy.init_node('move_to_server')
    server = MoveToServer()
    rospy.spin()


if __name__ == '__main__':
    main()
