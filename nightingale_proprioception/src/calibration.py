#!/usr/bin/env python3

import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


class TorqueCalibration:
    def __init__(self, arm_side):
        self.arm_side = arm_side

        self.xdata = []
        self.ydata = []

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
            f"/nightingale/{self.arm_side}_arm/computed_torques", JointState
        )

        x = [
            effort
            for joint, effort in zip(joint_state.name, joint_state.effort)
            if "measured" in joint
        ]
        y = [
            effort
            for joint, effort in zip(joint_state.name, joint_state.effort)
            if "computed" in joint
        ]

        rospy.loginfo(f"{x} {y}")

        self.xdata.append(
            [
                effort
                for joint, effort in zip(joint_state.name, joint_state.effort)
                if "measured" in joint
            ]
        )
        self.ydata.append(
            [
                effort
                for joint, effort in zip(joint_state.name, joint_state.effort)
                if "computed" in joint
            ]
        )

        rospy.loginfo(f"Recorded {len(self.xdata)} points")

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
        ManipulationControl,
    )

    rospy.init_node("torque_calibration_node")

    manipulation = ManipulationControl()

    torque_calibration = TorqueCalibration("right")

    rospy.loginfo("Right arm calibration:")
    right_joint_p1 = np.array(
        [
            -1.497173771091874,
            -0.05384432355144005,
            -0.0472769683033043,
            -1.4575817535881566,
            3.1601009435928535,
            1.2459021253979168,
            1.515535734175753,
        ]
    )
    right_joint_p2 = np.array(
        [
            -0.8926047783430242,
            -1.8371089736934032,
            9.374327037114938e-05,
            -2.2107645893852976,
            0.9909354102705024,
            1.2854942760596888,
            2.862406818614062,
        ]
    )

    torque_calibration.record(True)
    for iter in range(10):
        shuffle = np.random.uniform(size=7)
        joint_posns = [
            shuffle[idx] * right_joint_p1[idx]
            + (1 - shuffle[idx]) * right_joint_p2[idx]
            for idx in range(7)
        ]
        rospy.loginfo(f"{joint_posns}")

        manipulation.jnt_ctrl.cmd_right_arm(joint_posns)
        rospy.sleep(1)
        torque_calibration.record(True)

    weights, inv_noise_cov = torque_calibration.analyze(True)
    torque_calibration.clear(True)

    torque_calibration.arm_side = "left"

    rospy.loginfo("Left arm calibration:")
    left_joint_p1 = np.array(
        [
            0.7075179454806886,
            1.1448934852572825,
            1.334183252372954,
            1.987902747251979,
            -2.147573103039898,
            0.8198781100307042,
            -2.8961557275280905,
        ]
    )
    left_joint_p2 = np.array(
        [
            2.1888951099877305,
            0.6460906036203844,
            -0.4657128387765539,
            1.7709637753829963,
            -2.1124188440186336,
            -1.2119725210039258,
            -2.8808196480747608,
        ]
    )

    torque_calibration.record(True)
    for iter in range(10):
        shuffle = np.random.uniform(size=7)
        joint_posns = [
            shuffle[idx] * left_joint_p1[idx] + (1 - shuffle[idx]) * left_joint_p2[idx]
            for idx in range(7)
        ]
        rospy.loginfo(f"{joint_posns}")

        manipulation.jnt_ctrl.cmd_left_arm(joint_posns)
        rospy.sleep(1)
        torque_calibration.record(True)

    weights, inv_noise_cov = torque_calibration.analyze(True)
    torque_calibration.clear(True)
