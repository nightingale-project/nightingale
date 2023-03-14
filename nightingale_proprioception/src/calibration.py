#!/usr/bin/env python

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class TorqueCalibration:
    def __init__(self, record_func):
        rospy.init_node("torque_calibration_node")

        self.measured_torques = np.zeros(7)
        self.computed_torques = np.zeros(7)

        self.xdata = []
        self.ydata = []

        self.record_func = record_func

        self.record_sub = rospy.Subscriber(
            "/nightingale/torque_calibration/record", Bool, self.record
        )
        self.clear_sub = rospy.Subscriber(
            "/nightingale/torque_calibration/clear", Bool, self.clear
        )
        self.analyze_sub = rospy.Subscriber(
            "/nightingale/torque_calibration/analyze", Bool, self.analyze
        )

    def record(self, msg):
        joint_state = rospy.wait_for_message(
            "/nightingale/proprioception/computed_torques", JointState
        )

        self.xdata.append(
            [
                effort
                for joint, effort in zip(joint_state.name, joint_state.effort)
                if "measured" in joint_state.name
            ]
        )
        self.ydata.append(
            [
                effort
                for joint, effort in zip(joint_state.name, joint_state.effort)
                if "computed" in joint_state.name
            ]
        )

    def clear(self, msg):
        self.xdata = []
        self.ydata = []

    def analyze(self, msg):
        x = np.array(self.xdata)
        y = np.array(self.ydata)
        N, M = x.shape

        weights = np.zeros((7, 2))
        error_deviations = np.zeros((7, N))

        for idx in range(M):
            xi = x[:, idx]
            yi = y[:, idx]

            Xi = np.stack((xi, np.ones(N))).T
            Yi = np.copy(yi)

            Wi = np.linalg.inv(Xi.T @ Xi) @ Xi.T @ Yi

            weights[idx, :] = Wi

            err = yi - Xi @ Wi
            error_deviations[idx, :] = err - np.mean(err)
        error_covariance = error_deviations @ error_deviations.T / (N - 1)

        rospy.loginfo(
            f"Torque weights T_scaled[i] = w[i, 0] * T_raw + w[i, 1]\n{weights}"
        )
        rospy.loginfo(f"Error model covariance\n{error_covariance}")

        return (weights, np.linalg.inv(error_covariance))


if __name__ == "__main__":
    from nightingale_manipulation.manipulation_action_client import (
        ManipulationJointControl,
    )

    torque_calibration = TorqueCalibration()
    manipulation = ManipulationJointControl()

    rospy.loginfo("Right arm calibration:")
    right_joint_positions = [
        [
            -1.497173771091874,
            -0.05384432355144005,
            -0.0472769683033043,
            -1.4575817535881566,
            3.1601009435928535,
            1.2459021253979168,
            1.515535734175753,
        ],
        [
            -0.8926047783430242,
            -1.8371089736934032,
            9.374327037114938e-05,
            -2.2107645893852976,
            0.9909354102705024,
            1.2854942760596888,
            2.862406818614062,
        ],
    ]

    for joint_posns in right_joint_positions:
        manipulation.cmd_right_arm(joint_posns)
        torque_calibration.record()

    weights, inv_noise_cov = torque_calibration.analyze()
    torque_calibration.clear()

    rospy.loginfo("Left arm calibration:")
    left_joint_positions = [
        [
            -1.497173771091874,
            -0.05384432355144005,
            -0.0472769683033043,
            -1.4575817535881566,
            3.1601009435928535,
            1.2459021253979168,
            1.515535734175753,
        ],
        [
            -0.8926047783430242,
            -1.8371089736934032,
            9.374327037114938e-05,
            -2.2107645893852976,
            0.9909354102705024,
            1.2854942760596888,
            2.862406818614062,
        ],
    ]

    for joint_posns in left_joint_positions:
        manipulation.cmd_left_arm(joint_posns)
        torque_calibration.record()

    weights, inv_noise_cov = torque_calibration.analyze()
    torque_calibration.clear()
