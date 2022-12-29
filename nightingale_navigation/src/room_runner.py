#!/usr/bin/env python3
import os.path
import rospy
from nightingale_msgs.msg import (
    RoomRunnerAction,
    RoomRunnerResult,
    RoomRunnerFeedback,
)
from actionlib import SimpleActionServer, SimpleActionClient
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf_conversions
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np


class RoomRunnerNode(object):
    def __init__(self):
        self.logger_name = "room_runner"
        rospy.loginfo(
            "Starting the Nightingale Navigation Room Runner Node!",
            logger_name=self.logger_name,
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.save_pose = rospy.get_param("/room_runner/save_pose", False)
        self.pose_filename = rospy.get_param("/room_runner/pose_filename", "")
        self.pose_write_period = rospy.get_param("/room_runner/pose_write_period", 10)

        """
        Cached pose file has the expected form: x y theta
        """
        self.EXPECTED_NUM_LINES = 1
        self.EXPECTED_NUM_VALUES = 3
        # this is the cov matrix rviz uses when we supply an initial pose
        self.INITAL_COV = np.reshape(
            np.diag([0.25, 0.25, 0, 0, 0, 0.06853892]), (36,)
        ).tolist()

        if self.save_pose and not os.path.isabs(self.pose_filename):
            rospy.logfatal(
                f"Pose file path {self.pose_filename} is not absolute! Please provide an absolute path",
                logger_name=self.logger_name,
            )

        if self.save_pose and os.path.exists(self.pose_filename):
            x, y, yaw = self.read_pose()
            self.publish_initial_pose(x, y, yaw)
        elif self.save_pose:
            rospy.logwarn(
                "Pose file path does not yet exist, It will be written",
                logger_name=self.logger_name,
            )

        self.server = SimpleActionServer(
            "/nightingale_navigation/room_runner",
            RoomRunnerAction,
            self.execute_cb,
            False,
        )
        self.server.register_preempt_callback(self.preempt_cb)
        self.server.start()

        move_base_action_name = "move_base"
        self.client = SimpleActionClient(move_base_action_name, MoveBaseAction)
        rospy.loginfo(
            f"Waiting for {move_base_action_name} action server",
            logger_name=self.logger_name,
        )
        if self.client.wait_for_server():
            rospy.loginfo(
                f"Found the {move_base_action_name} action server",
                logger_name=self.logger_name,
            )
        else:
            rospy.logfatal(
                f"Failed to find the {move_base_action_name} action server",
                logger_name=self.logger_name,
            )

        self.timer = (
            rospy.Timer(rospy.Duration(self.pose_write_period), self.write_pose)
            if self.save_pose
            else None
        )

        self.home = "home"

    def execute_cb(self, goal):
        rospy.loginfo(
            f"Received room_runner action call. Navigating to {goal.room_number} with sublocation {goal.sublocation}",
            logger_name=self.logger_name,
        )
        goal_pose = self.lookup_pose(goal.room_number, goal.sublocation)
        msg, fb_cb, a_cb = self.generate_goal_info(goal_pose, "navigating to door")

        self.client.send_goal(msg, feedback_cb=fb_cb, active_cb=a_cb)
        self.client.wait_for_result()

        # move base result is empty so no need to check
        result = RoomRunnerResult()
        if self.client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo(
                "RoomRunner Completed Successfully :)", logger_name=self.logger_name
            )
            self.server.set_succeeded(result)
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            rospy.logwarn(
                "RoomRunner preempted by caller", logger_name=self.logger_name
            )
        else:
            rospy.logerr(
                "RoomRunner Failed. See return code", logger_name=self.logger_name
            )
            # the action call succeeded but the actual action performance failed
            # this is why we call set_succeeded
            self.server.set_succeeded(result)
        return

    def preempt_cb(self):
        rospy.logwarn("Cancelling room_runner actions", logger_name=self.logger_name)
        self.client.cancel_all_goals()
        result = RoomRunnerResult()
        self.server.set_preempted(result)

    def generate_goal_info(self, pose, phase):
        msg = MoveBaseGoal()
        msg.target_pose = pose

        def fb_cb(fb):
            target = np.array([pose.pose.position.x, pose.pose.position.y])
            cur = np.array(
                [fb.base_position.pose.position.x, fb.base_position.pose.position.y]
            )
            euclidean_distance = np.linalg.norm(target - cur)
            ret = RoomRunnerFeedback()
            ret.euclidean_distance_to_goal = euclidean_distance
            self.server.publish_feedback(ret)

        a_cb = lambda: rospy.loginfo(
            "Move Base activated the action", logger_name=self.logger_name
        )
        return msg, fb_cb, a_cb

    def publish_initial_pose(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance = self.INITAL_COV.copy()

        pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)
        # Fast rate for checking if amcl is up
        rate = rospy.Rate(100)
        while pub.get_num_connections() == 0:
            rospy.loginfo_throttle(5, "Waiting for amcl to subscribe to /initialpose")
            rate.sleep()
        pub.publish(msg)

    def lookup_pose(self, name, subloc):
        # TODO Service Call, currently, just always returns the same pose
        ret = PoseStamped()
        ret.header.stamp = rospy.Time.now()
        ret.header.frame_id = "map"
        ret.pose.position.x = 4.80
        ret.pose.position.z = 0.0
        if subloc == "doorside":
            ret.pose.position.y = 12.24
        else:
            ret.pose.position.y = -3.34
        ret.pose.orientation.x = 0.0
        ret.pose.orientation.y = 0.0
        ret.pose.orientation.z = 0.0
        ret.pose.orientation.w = 1.0
        return ret

    def read_pose(self):
        with open(self.pose_filename, "r") as f:
            file_data = list(f)
            if len(file_data) != self.EXPECTED_NUM_LINES:
                rospy.logfatal(
                    f"Corruputed pose cached file {self.pose_filename}. Please delete.",
                    logger_name=self.logger_name,
                )
            ret = [float(part) for part in file_data[0].split(" ")]
            if len(ret) != self.EXPECTED_NUM_VALUES:
                rospy.logfatal(
                    f"Corruputed pose cached file {self.pose_filename}. Please delete.",
                    logger_name=self.logger_name,
                )
            return ret

    def write_pose(self, time=None):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0))
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(
                "Failed to look up transform from map to base_link",
                logger_name=self.logger_name,
            )
            return
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        quat_msg = trans.transform.rotation
        quat = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        yaw = tf_conversions.transformations.euler_from_quaternion(quat)[-1]
        with open(self.pose_filename, "w") as f:
            f.write(f"{x} {y} {yaw}")


def main():
    rospy.init_node("room_runner")
    rr = RoomRunnerNode()
    if rr.save_pose:
        rospy.on_shutdown(lambda: rr.write_pose())
    rospy.spin()


if __name__ == "__main__":
    main()
