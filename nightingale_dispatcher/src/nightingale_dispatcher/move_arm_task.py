#!/usr/bin/env python3
import actionlib
from actionlib_msgs.msg import GoalStatus
import rospy
from nightingale_dispatcher.task import Task, TaskCodes
from nightingale_manipulation.manipulation_action_client import ManipulationControl
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped
import tf2_ros
import numpy as np


class MoveArmTask(Task):
    def __init__(self):
        self.manipulation = ManipulationControl()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.right_arm_length = 0.8  # this is conservative

        self.manipulation.right_cartesian.set_ref_link("upper_body_link")
        self.manipulation.open_right_gripper()
        rospy.sleep(2.0)
        self.manipulation.close_right_gripper()

        if not self.manipulation.home_left() or not self.manipulation.home_right():
            rospy.logfatal("MoveArmTask: Failed to home arms. This is unrecoverable")
        self.manipulation.right_cartesian.set_ref_link("base_link")

    def extend_restock(self):
        result = self.manipulation.extend_restock()
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR

    def retract_right_arm(self):
        result = self.manipulation.retract_right()
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR

    def extend_handoff(self, point):
        result = self.manipulation.extend_handoff(point)
        assert type(result) is bool
        return TaskCodes.SUCCESS if result else TaskCodes.ERROR

    # this function returns true if a given point is within the right arm's workspace
    def within_workspace(self, point):
        assert type(point) is Point
        # assume the point is in the base_link frame
        stamped_point = PointStamped()
        stamped_point.point = point
        stamped_point.header.frame_id = "base_link"
        try:
            stamped_point = self.tf_buffer.transform(
                stamped_point, "right_shoulder_link", rospy.Duration((3))
            )
        except tf2_ros.TransformException as ex:
            rospy.logerr(
                f"ArmTask failed to transform stamped_point from base_link to right_shoulder_link. Exception: {ex}"
            )
            return False
        return (
            np.linalg.norm(
                [stamped_point.point.x, stamped_point.point.y, stamped_point.point.z]
            )
            < self.right_arm_length
        )
