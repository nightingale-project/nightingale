import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction
)

trajectory_msgs/JointTrajectoryPoint.msg

class TrajectoryInterceptServer:
    def __init__(self, action_server):
        self.action_client = rospy.Publisher(
            action_server, FollowJointTrajectoryActionGoal, queue_size=10
        )
        self.goal_interceptor = rospy.Subscriber(
            action_server, FollowJointTrajectoryActionGoal, self.goal_interceptor_cb
        )
        self.previous_trajectory = None

    def goal_interceptor_cb(self, goal):
        pass

    def send_reverse_trajectory(self):
        pass
