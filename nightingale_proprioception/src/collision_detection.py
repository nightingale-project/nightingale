#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from urdf_parser_py.urdf import Robot


class CollisionDetector:
    GRAVITATIONAL_ACCELERATION = 9.8  # [m/sec^2]
    COLLISION_DETECTION_THRESHOLD = 50  # std dev

    def __init__(self, arm_side, box_mass):
        rospy.init_node("collision_detection_node")

        self.robot = Robot.from_parameter_server()

        self.dof = 7
        self.arm_side = arm_side
        self.box_mass = box_mass
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
                        1.08250411e06,
                        -1.35973568e06,
                        -1.08006283e07,
                        -1.96282654e06,
                        -1.92171379e06,
                        1.12331074e08,
                        -6.00798205e21,
                    ],
                    [
                        -1.35973568e06,
                        1.70796697e06,
                        1.35666927e07,
                        2.46551045e06,
                        2.41386878e06,
                        -1.41099305e08,
                        7.54663934e21,
                    ],
                    [
                        -1.08006283e07,
                        1.35666927e07,
                        1.07762715e08,
                        1.95839973e07,
                        1.91737995e07,
                        -1.12077752e09,
                        5.99443328e22,
                    ],
                    [
                        -1.96282654e06,
                        2.46551045e06,
                        1.95839973e07,
                        3.55905354e06,
                        3.48450479e06,
                        -2.03681809e08,
                        1.08938389e22,
                    ],
                    [
                        -1.92171379e06,
                        2.41386878e06,
                        1.91737995e07,
                        3.48450479e06,
                        3.41151965e06,
                        -1.99415573e08,
                        1.06656613e22,
                    ],
                    [
                        1.12331074e08,
                        -1.41099305e08,
                        -1.12077752e09,
                        -2.03681809e08,
                        -1.99415573e08,
                        1.16565572e10,
                        -6.23446249e23,
                    ],
                    [
                        -6.00798205e21,
                        7.54663934e21,
                        5.99443328e22,
                        1.08938389e22,
                        1.06656613e22,
                        -6.23446249e23,
                        3.33447709e37,
                    ],
                ]
            ),
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lookup_gravity_tf()

        self.collision_pub = rospy.Publisher(
            f"/nightingale/proprioception/collision", Bool, queue_size=10
        )

        self.computed_torques_pub = rospy.Publisher(
            f"/nightingale/proprioception/computed_torques", JointState, queue_size=10
        )

        self.iter = 0
        self.iter_decimation = 10

        self.joint_state_sub = rospy.Subscriber(
            f"/movo/{self.arm_side}_arm/joint_states", JointState, self.joint_state_cb
        )

    def joint_state_cb(self, msg):
        if self.iter == self.iter_decimation - 1:
            self.lookup_tf()

            self.get_jacobian()
            self.get_gravity()

            self.estimate_torques(msg.effort[: self.dof])
            self.detect_collisions()
            self.pub_torques()

            self.iter = 0
        else:
            self.iter += 1

    def estimate_torques(self, torques):
        self.measured_torques = self.scale_torques(torques)
        self.computed_torques = self.jacobian[:3, :].T @ self.gravity_forces[:3]

        # Replace right elbow measurement because the sensor is broken
        if self.arm_side == "right":
            self.measured_torques[3] = self.computed_torques[3]

    def detect_collisions(self):
        error = (self.measured_torques - self.computed_torques).reshape((self.dof, 1))
        error_distance = np.sqrt(
            error[:6, 0].T @ self.error_models[self.arm_side][:6, :6] @ error[:6, 0]
        )

        collision_detected = np.any(error_distance > self.COLLISION_DETECTION_THRESHOLD)
        if collision_detected:
            self.collision_pub.publish(Bool(collision_detected))

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

            if idx == self.dof and self.box_mass is not None:
                link_mass = self.robot.link_map[to_link].inertial.mass + self.box_mass
            else:
                link_mass = self.robot.link_map[to_link].inertial.mass

            gravity_forces[:3] += (
                link_mass * self.GRAVITATIONAL_ACCELERATION * self.gravity_direction
            )
            gravity_forces[3:] += link_mass * np.cross(
                self.gravity_direction, self.translations[idx]
            )

        self.gravity_forces = gravity_forces

    def scale_torques(self, torques):
        return (
            self.weights[self.arm_side][:, 0] * np.array(torques)
            + self.weights[self.arm_side][:, 1]
        )

    def pub_torques(self):
        joint_states = JointState()
        joint_states.name = [f"measured_joint_{idx}" for idx in range(self.dof)] + [
            f"computed_joint_{idx}" for idx in range(self.dof)
        ]
        joint_states.effort = (
            self.measured_torques.tolist() + self.computed_torques.tolist()
        )
        self.computed_torques_pub.publish(joint_states)


if __name__ == "__main__":
    collision_detector = CollisionDetector("right", 1.2)
    rospy.spin()
