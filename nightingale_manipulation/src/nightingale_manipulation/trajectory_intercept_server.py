import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)


class TrajectoryInterceptServer:
    def __init__(self, action_server):
        self.action_client = actionlib.SimpleActionClient(
            action_server, FollowJointTrajectoryAction
        )
        self.action_client.wait_for_server()
        self.goal_interceptor = rospy.Subscriber(
            action_server + "/goal",
            FollowJointTrajectoryActionGoal,
            self.goal_interceptor_cb,
        )
        self.inverted_trajectory = None
        self.previous_path_tolerance = []
        self.previous_goal_tolerance = []
        self.previous_duration = rospy.Duration(0.0)

    def goal_interceptor_cb(self, goal):
        intercepted_trajectory = goal.goal.trajectory

        inverted_position_list = [
            point.positions for point in reversed(intercepted_trajectory.points)
        ]
        negative_velocities_list = [
            -1 * point.velocities for point in reversed(intercepted_trajectory.points)
        ]
        negative_acceleration_list = [
            -1 * point.accelerations
            for point in reversed(intercepted_trajectory.points)
        ]

        for index, point in enumerate(intercepted_trajectory.points):
            point.positions = inverted_position_list[index]
            point.velocities = negative_velocities_list[index]
            point.accelerations = negative_acceleration_list[index]

        self.inverted_trajectory = intercepted_trajectory
        self.previous_goal_tolerance = goal.goal.goal_tolerance
        self.previous_path_tolerance = goal.goal.path_tolerance
        self.previous_duration = goal.goal.goal_time_tolerance

    def send_reverse_trajectory(self):
        if not self.inverted_trajectory:
            return False
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = self.inverted_trajectory
        goal.path_tolerance = self.previous_path_tolerance
        goal.goal_tolerance = self.previous_goal_tolerance
        goal.goal_time_tolerance = self.previous_duration
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()
