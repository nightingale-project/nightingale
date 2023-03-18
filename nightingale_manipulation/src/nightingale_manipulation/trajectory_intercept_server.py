import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


fet_p1 = JointTrajectoryPoint()
fet_p1.positions = []
fet_p1.velocities = []
fet_p1.accelerations = []
fet_p1.time_from_start = rospy.duration()

fet_p2 = JointTrajectoryPoint()
fet_p2.positions = []
fet_p2.velocities = []
fet_p2.accelerations = []
fet_p2.time_from_start = rospy.duration()

fast_extend_trajectory = JointTrajectory()
fast_extend_trajectory.points = [fet_p1, fet_p1]


def invert_trajectory(trajectory):
    inverted_position_list = [
        point.positions for point in reversed(trajectory.points)
    ]
    negative_velocities_list = [
        -1 * point.velocities for point in reversed(trajectory.points)
    ]
    negative_acceleration_list = [
        -1 * point.accelerations
        for point in reversed(trajectory.points)
    ]

    for index, point in enumerate(trajectory.points):
        point.positions = inverted_position_list[index]
        point.velocities = negative_velocities_list[index]
        point.accelerations = negative_acceleration_list[index]

    return trajectory


fast_retract_trajectory = invert_trajectory(fast_extend_trajectory)


class TrajectoryInterceptServer:
    def __init__(self, action_server, debug=False):
        self.action_client = actionlib.SimpleActionClient(
            action_server, FollowJointTrajectoryAction
        )
        self.action_client.wait_for_server()
        self.goal_interceptor = rospy.Subscriber(
            action_server + "/goal",
            FollowJointTrajectoryActionGoal,
            self.goal_interceptor_cb,
        )
        self.debug = debug
        self.inverted_trajectory = None
        self.previous_path_tolerance = []
        self.previous_goal_tolerance = []
        self.previous_duration = rospy.Duration(0.0)

    def goal_interceptor_cb(self, goal):
        intercepted_trajectory = goal.goal.trajectory
        if self.debug:
            print("positions: ")
            print(point.positions for point in intercepted_trajectory.points)
            print("velocities: ")
            print(point.velocities for point in intercepted_trajectory.points)
            print("accelerations: ")
            print(point.accelerations for point in intercepted_trajectory.points)
            print("durations: ")
            print(point.time_from_start for point in intercepted_trajectory.points)

        self.inverted_trajectory = invert_trajectory(intercepted_trajectory)
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

    def fast_extend(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = fast_extend_trajectory
        goal.path_tolerance = self.previous_path_tolerance
        goal.goal_tolerance = self.previous_goal_tolerance
        goal.goal_time_tolerance = self.previous_duration
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    def fast_retract(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = fast_retract_trajectory
        goal.path_tolerance = self.previous_path_tolerance
        goal.goal_tolerance = self.previous_goal_tolerance
        goal.goal_time_tolerance = self.previous_duration
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()
