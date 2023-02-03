#!/usr/bin/env python3

import rospy
from nightingale_dispatcher.task import Task
import actionlib
from actionlib_msgs.msg import GoalStatus


class MoveArmTask(Task):
    def __init__(self):
        # rospy.loginfo(f"Waiting for {action_name} action server")
        ## add arm action stuff
        # if self.action_client.wait_for_server():
        #    rospy.loginfo(
        #        f"Found the {action_name} action server",
        #    )
        # else:
        #    rospy.logfatal(
        #        f"Failed to find the {action_name} action server",
        #    )
        pass

    def execute(self, end_pose):
        # do arm stuff
        return Task.Success
