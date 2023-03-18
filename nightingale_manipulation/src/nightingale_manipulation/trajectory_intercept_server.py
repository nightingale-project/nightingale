import rospy
import actionlib
import copy
from control_msgs.msg import (
    FollowJointTrajectoryActionGoal,
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


fet_p1 = JointTrajectoryPoint()
fet_p2 = JointTrajectoryPoint()
fet_p3 = JointTrajectoryPoint()
fet_p4 = JointTrajectoryPoint()
fet_p1.positions = [
    0.0007760451416487513,
    -2.21005392484841,
    -1.8376187027260444,
    -0.8936141163961651,
    2.8625341177141674,
    0.9902415902275088,
    1.2848500573919988,
]
fet_p2.positions = [
    -0.1698596616115808,
    -2.1835711833348816,
    -1.423859968505162,
    -0.6786463086788783,
    2.4871662313961633,
    0.8796995135257663,
    1.076168415696402,
]
fet_p3.positions = [
    -0.3966680811413077,
    -1.9131534170545033,
    -1.0805514237592584,
    -0.4084431837849145,
    2.336356661663678,
    0.7566377214210307,
    1.025690501771925,
]
fet_p4.positions = [
    -0.7019779612169726,
    -1.4488055757410898,
    -0.7940901867559821,
    -0.10198352235519945,
    2.3112663654843493,
    0.6924912247123424,
    1.168862723630301,
]
fet_p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fet_p2.velocities = [
    -0.13413232809845205,
    0.10646083840862161,
    0.25,
    0.1633256133332604,
    -0.16831216262429147,
    -0.07820297841462029,
    -0.08142369059618382,
]
fet_p3.velocities = [
    -0.1951004742348621,
    0.26959051841311454,
    0.23057216883815548,
    0.21132439761455737,
    -0.06415711664275114,
    -0.06844777829469914,
    0.034385354714466285,
]
fet_p4.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
fet_p1.accelerations = [0.0, 0.0, 0.0, 0.1569600365875225, 0.0, 0.0, 0.0]
fet_p2.accelerations = [
    -0.0409889046022985,
    0.1194867645246312,
    0.0,
    0.0441683878551222,
    0.07726055758052161,
    -0.015073419629824648,
    0.05899776059742704,
]
fet_p3.accelerations = [
    -0.04386435228902297,
    0.10647867596697598,
    -0.0284662883651982,
    0.021334286936912437,
    0.0669077964503675,
    0.031014271513192963,
    0.10424216117334352,
]
fet_p4.accelerations = [
    0.3317401273039688,
    -0.5045457813302944,
    -0.3112597836911396,
    -0.33298944361390864,
    0.027262327857067628,
    0.06969956877573064,
    -0.15556643988697272,
]
fet_p1.time_from_start = rospy.Duration(0)
fet_p2.time_from_start = rospy.Duration(1.655034937)
fet_p3.time_from_start = rospy.Duration(3.28259116)
fet_p4.time_from_start = rospy.Duration(4.384977234)

fast_extend_trajectory = JointTrajectory()
fast_extend_trajectory.joint_names = [
    "right_arm_half_joint",
    "right_elbow_joint",
    "right_shoulder_lift_joint",
    "right_shoulder_pan_joint",
    "right_wrist_3_joint",
    "right_wrist_spherical_1_joint",
    "right_wrist_spherical_2_joint",
]
fast_extend_trajectory.header.frame_id = "odom"
fast_extend_trajectory.points = [fet_p1, fet_p2, fet_p3, fet_p4]


def invert_trajectory(trajectory):
    inverted_position_list = [point.positions for point in reversed(trajectory.points)]
    negative_velocities_list = [
        -1 * point.velocities for point in reversed(trajectory.points)
    ]
    negative_acceleration_list = [
        -1 * point.accelerations for point in reversed(trajectory.points)
    ]

    for index, point in enumerate(trajectory.points):
        point.positions = inverted_position_list[index]
        point.velocities = negative_velocities_list[index]
        point.accelerations = negative_acceleration_list[index]

    return trajectory


fast_retract_trajectory = invert_trajectory(copy.deepcopy(fast_extend_trajectory))
fast_retract_trajectory.joint_names = [
    "right_arm_half_joint",
    "right_elbow_joint",
    "right_shoulder_lift_joint",
    "right_shoulder_pan_joint",
    "right_wrist_3_joint",
    "right_wrist_spherical_1_joint",
    "right_wrist_spherical_2_joint",
]
fast_retract_trajectory.header.frame_id = "odom"


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
            print([point.positions for point in intercepted_trajectory.points])
            print("velocities: ")
            print([point.velocities for point in intercepted_trajectory.points])
            print("accelerations: ")
            print([point.accelerations for point in intercepted_trajectory.points])
            print("durations: ")
            print([point.time_from_start for point in intercepted_trajectory.points])

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
        rospy.loginfo(f"Extend {fast_extend_trajectory}")
        goal.path_tolerance = self.previous_path_tolerance
        goal.goal_tolerance = self.previous_goal_tolerance
        goal.goal_time_tolerance = self.previous_duration
        rospy.loginfo(f"Extend goal\n{goal}")
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()

    def fast_retract(self):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = fast_retract_trajectory
        rospy.loginfo(f"Retract {fast_retract_trajectory}")
        goal.path_tolerance = self.previous_path_tolerance
        goal.goal_tolerance = self.previous_goal_tolerance
        goal.goal_time_tolerance = self.previous_duration
        rospy.loginfo(f"Retract goal\n{goal}")
        self.action_client.send_goal(goal)
        self.action_client.wait_for_result()
        return self.action_client.get_result()
