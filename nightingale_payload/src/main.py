#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from nightingale_msgs.msg import Payload
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from urdf_parser_py.urdf import Robot


class SensorCalibration:
    def __init__(self, record_func):
        self.xdata = []
        self.ydata = []

        self.record_func = record_func

        self.record_sub = rospy.Subscriber("analysis/record", Bool, self.record)
        self.clear_sub = rospy.Subscriber("analysis/clear", Bool, self.clear)
        self.analyze_sub = rospy.Subscriber("analysis/analyze", Bool, self.analyze)

        self.weights = None

    def record(self, msg):
        x, y = self.record_func()

        self.xdata.append(list(x))
        self.ydata.append(list(y))

        rospy.loginfo(f"Recorded:\nx={self.xdata}\ny={self.ydata}")

    def clear(self, msg):
        self.xdata = []
        self.ydata = []

        rospy.loginfo("Cleared")

    def analyze(self, msg):
        x = np.array(self.xdata)
        y = np.array(self.ydata)
        N, M = x.shape

        rospy.loginfo(f"Analyzed:\nx={x}\ny={y}")

        weights = np.zeros((7, 2))

        for idx in range(M):
            X = np.stack((x[:, idx], np.ones(N))).T
            Y = y[:, idx]

            rospy.loginfo(f"{X} {Y}")
            weights[idx, :] = np.linalg.inv(X.T @ X) @ X.T @ Y

        rospy.loginfo(f"weights: {weights}")

        self.weights = weights

        np.savez("/home/lyndon/Documents/mte_4812/data.npz", x=x, y=y)


class PayloadEstimator:
    GRAVITATIONAL_ACCELERATION = 9.8  # [m/sec^2]
    PAYLOAD_DETECTION_THRESHOLD = 50  # std dev

    def __init__(self, arm_side):
        rospy.init_node("payload_estimator_node")

        self.robot = Robot.from_parameter_server()

        self.dof = 7
        self.arm_side = arm_side
        self.joint_link_suffixes = [
            "base_link",
            "shoulder_link",
            "arm_half_1_link",
            "arm_half_2_link",
            "forearm_link",
            "wrist_spherical_1_link",
            "wrist_spherical_2_link",
            "wrist_3_link",
            "ee_link",
            "gripper_base_link",
            "gripper_finger1_knuckle_link",
            "gripper_finger1_finger_tip_link",
            "gripper_finger2_knuckle_link",
            "gripper_finger2_finger_tip_link",
            "gripper_finger3_knuckle_link",
            "gripper_finger3_finger_tip_link",
        ]

        self.rotation_axes = np.zeros((self.dof, 3))
        self.translations = np.zeros((self.dof, 3))

        self.jacobian = np.zeros((6, self.dof))
        self.base_link_rotation = np.zeros((3, 3))
        self.gravity_direction = np.zeros(3)
        self.gravity_forces = np.zeros(6)

        self.weights = {
            "right": np.zeros((self.dof, 2)),
            "left": np.array(
                [
                    [-2.44161661e00, -8.80378258e00],
                    [-2.11674602e00, -3.00928765e00],
                    [-3.19820241e00, -3.09657843e00],
                    [-3.93661567e00, 2.14572217e00],
                    [-2.28678270e00, -7.84129984e00],
                    [-7.25424816e00, -6.86924505e-01],
                    [-2.16260351e-14, 1.31909042e-14],
                ]
            ),
        }
        self.error_models = {
            "right": np.zeros((self.dof, self.dof)),
            "left": np.array(
                [
                    [
                        5.68075404e01,
                        -1.24970088e01,
                        2.49631265e00,
                        3.24242928e00,
                        1.80881417e01,
                        -5.92326271e-01,
                        -9.34355742e-15,
                    ],
                    [
                        -1.24970088e01,
                        2.96429465e01,
                        -1.58984130e-01,
                        -2.08735398e00,
                        -8.15410739e00,
                        1.20005465e-01,
                        -3.14085321e-15,
                    ],
                    [
                        2.49631265e00,
                        -1.58984130e-01,
                        5.55697289e00,
                        7.44006773e-01,
                        5.52154569e-01,
                        1.74704528e-01,
                        -6.65731777e-15,
                    ],
                    [
                        3.24242928e00,
                        -2.08735398e00,
                        7.44006773e-01,
                        9.03829531e-01,
                        4.42269859e-01,
                        2.41429762e-02,
                        -2.66232176e-16,
                    ],
                    [
                        1.80881417e01,
                        -8.15410739e00,
                        5.52154569e-01,
                        4.42269859e-01,
                        1.10720364e01,
                        -1.64741943e-01,
                        -2.65425581e-15,
                    ],
                    [
                        -5.92326271e-01,
                        1.20005465e-01,
                        1.74704528e-01,
                        2.41429762e-02,
                        -1.64741943e-01,
                        2.80167501e-02,
                        1.20682787e-16,
                    ],
                    [
                        -9.34355742e-15,
                        -3.14085321e-15,
                        -6.65731777e-15,
                        -2.66232176e-16,
                        -2.65425581e-15,
                        1.20682787e-16,
                        1.41876623e-29,
                    ],
                ]
            ),
        }

        self.iter = 0
        self.iter_decimation = 10

        self.forces = np.zeros(6)
        self.last_forces = np.zeros(6)
        self.filter_coeff = 1

        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lookup_gravity_tf()

        self.analysis = SensorCalibration(self.calibration_record_cb)
        self.measured_torques = np.zeros(7)
        self.computed_torques = np.zeros(7)

        self.payload_pub = rospy.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )

        self.joint_state_sub = rospy.Subscriber(
            f"/movo/{self.arm_side}_arm/joint_states", JointState, self.joint_state_cb
        )

    def calibration_record_cb(self):
        return (self.measured_torques, self.computed_torques)

    def scale_torques(self, torques):
        return (
            self.weights[self.arm_side][:, 0] * np.array(torques)
            + self.weights[self.arm_side][:, 1]
        )

    def joint_state_cb(self, msg):
        if self.iter == self.iter_decimation - 1:
            self.lookup_tf()

            self.get_jacobian()
            self.get_gravity()

            torques = self.scale_torques(msg.effort[: self.dof])
            self.compare_torques(torques)

            self.iter = 0
        else:
            self.iter += 1

    def lookup_gravity_tf(self):
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                "base_link",
                f"{self.arm_side}_base_link",
                rospy.Time(0),
                rospy.Duration(20),
            )

            self.base_link_rotation = tf_conversions.transformations.quaternion_matrix(
                [
                    base_link_transform.transform.rotation.x,
                    base_link_transform.transform.rotation.y,
                    base_link_transform.transform.rotation.z,
                    base_link_transform.transform.rotation.w,
                ]
            )[:3, :3]
            self.gravity_direction = -(self.base_link_rotation.T)[:, 2]
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(
                f"Arm transform lookup from 'base_link' to {self.arm_side}_base_link failed:\n{e}"
            )

    def lookup_tf(self):
        rotation_axes = np.zeros((self.dof + 1, 3))
        translations = np.zeros((self.dof + 1, 3))

        for idx in range(self.dof + 1):
            from_link = f"{self.arm_side}_{self.joint_link_suffixes[0]}"
            to_link = f"{self.arm_side}_{self.joint_link_suffixes[idx + 1]}"
            try:
                link_transform = self.tf_buffer.lookup_transform(
                    from_link, to_link, rospy.Time()
                )

                rot_mat = tf_conversions.transformations.quaternion_matrix(
                    [
                        link_transform.transform.rotation.x,
                        link_transform.transform.rotation.y,
                        link_transform.transform.rotation.z,
                        link_transform.transform.rotation.w,
                    ]
                )
                rotation_axes[idx, :] = rot_mat[:3, 2]

                translations[idx, :] = [
                    link_transform.transform.translation.x,
                    link_transform.transform.translation.y,
                    link_transform.transform.translation.z,
                ]
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                rospy.logerr(
                    f"Arm transform lookup from {from_link} to {to_link} failed\n{e}"
                )

        self.rotation_axes = rotation_axes
        self.translations = translations

    def get_jacobian(self):
        jacobian = np.zeros((6, self.dof))

        for idx in range(self.dof):
            rotation_axis = self.rotation_axes[idx, :]
            ee_translation = self.translations[self.dof, :] - self.translations[idx, :]

            jacobian[:3, idx] = np.cross(rotation_axis, ee_translation)
            jacobian[3:, idx] = rotation_axis

        self.jacobian = jacobian

    def get_gravity(self):
        gravity_forces = np.zeros(6)

        for idx in range(self.dof + 1):
            to_link = f"{self.arm_side}_{self.joint_link_suffixes[idx]}"
            gravity_forces[:3] += (
                self.robot.link_map[to_link].inertial.mass
                * self.GRAVITATIONAL_ACCELERATION
                * self.gravity_direction
            )
            gravity_forces[3:] += self.robot.link_map[to_link].inertial.mass * np.cross(
                self.gravity_direction, self.translations[idx]
            )

        self.gravity_forces = gravity_forces

    def compare_torques(self, torques):
        computed_torques = self.jacobian[:3, :].T @ self.gravity_forces[:3]
        error = np.array(torques) - computed_torques
        error_norm = np.sqrt(np.sum(np.square(error)))

        # rospy.loginfo(
        #     f"{np.round(torques, 2)} {np.round(computed_torques)} {np.round(error_norm)}"
        # )

        # if self.analysis.weights is not None:
        #     calibrated_torques = (
        #         np.array(torques) * self.analysis.weights[:, 0]
        #         + self.analysis.weights[:, 1]
        #     )
        #     rospy.loginfo(
        #         f"{np.round(calibrated_torques, 2)} {np.round(computed_torques)}"
        #     )

        self.measured_torques = torques
        self.computed_torques = self.jacobian[:3, :].T @ self.gravity_forces[:3]

        error = (self.measured_torques - self.computed_torques).reshape((self.dof, 1))
        error_distance = np.sqrt(
            error[:6, 0].T
            @ np.linalg.inv(self.error_models[self.arm_side][:6, :6])
            @ error[:6, 0]
        )
        if np.any(error_distance > self.PAYLOAD_DETECTION_THRESHOLD):
            rospy.logwarn(f"COLLISION_DETECTED: {error_distance}")


if __name__ == "__main__":
    payload_estimator = PayloadEstimator("left")
    rospy.spin()
