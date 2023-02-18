#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from sensor_msgs.msg import JointState
from nightingale_msgs.msg import Payload
from urdf_parser_py.urdf import Robot


class PayloadEstimator:
    GRAVITATIONAL_ACCELERATION = -9.8  # [m/sec^2]
    PAYLOAD_DETECTION_THRESHOLD = 1  # [kg]

    def __init__(self, arm_side):
        rospy.init_node("payload_estimator_node")

        self.robot = Robot.from_parameter_server()
        rospy.loginfo(f"{self.robot.link_map['right_shoulder_link'].inertial}")

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
        self.gravity_forces = np.zeros(6)

        self.forces = np.zeros(6)
        self.last_forces = np.zeros(6)
        self.filter_coeff = 0.5

        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.payload_pub = rospy.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_cb
        )

    def joint_state_cb(self, msg):
        self.lookup_tf()

        self.get_jacobian()
        self.get_gravity()

        # 4: right_shoulder_pan_joint
        # 5: right_shoulder_lift_joint
        # 6: right_arm_half_joint
        # 7: right_elbow_joint
        # 8: right_wrist_spherical_1_joint
        # 9: right_wrist_spherical_2_joint
        # 10: right_wrist_3_joint
        self.compute_arm_forces(msg.effort[4:11])

    def lookup_tf(self):
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                "base_link", f"{self.arm_side}_base_link", rospy.Time()
            )

            rot_mat = tf_conversions.transformations.quaternion_matrix(
                [
                    link_transform.transform.rotation.x,
                    link_transform.transform.rotation.y,
                    link_transform.transform.rotation.z,
                    link_transform.transform.rotation.w,
                ]
            )
            self.gravity = self.GRAVITATIONAL_ACCELERATION * rot_mat[:3, 2]
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rospy.logerr(f"Arm transform lookup from {from_link} to {to_link} failed")

        rotation_axes = np.zeros((self.dof, 3))
        translations = np.zeros((self.dof, 3))

        for idx in range(self.dof):
            from_link = f"{self.arm_side}_{self.joint_link_suffixes[0]}"
            to_link = f"{self.arm_side}_{self.joint_link_names[idx]}"
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
            ):
                rospy.logerr(
                    f"Arm transform lookup from {from_link} to {to_link} failed"
                )

        self.rotation_axes = rotation_axes
        self.translations = translations

    def get_jacobian(self):
        jacobian = np.zeros((6, self.dof))

        for idx in range(self.dof):
            ee_translation = self.translations[-1, :] - self.translations[idx, :]
            rotation_axis = self.rotation_axes[idx, :]

            jacobian[:3, idx] = np.cross(rotation_axis, ee_translation)
            jacobian[3:, idx] = rotation_axis

        self.jacobian = jacobian

    def get_gravity(self):
        gravity_forces = np.zeros(6)

        for idx in range(self.dof):
            ee_translation = self.translations[-1, :] - self.translations[idx, :]

            to_link = f"{self.arm_side}_{self.joint_link_suffixes[idx]}"
            gravity_forces[:3] += (
                self.robot.link_map[to_link].inertial.mass * self.gravity
            )
            gravity_forces[3:] += self.robot.link_map[to_link].inertial.mass * np.cross(
                self.gravity, ee_translation
            )

        self.gravity_forces = gravity_forces

    def compute_arm_forces(self, torques):
        forces = np.linalg.pinv(self.jacobian.T) @ torques - self.gravity_forces

        self.forces = self.last_forces + self.filter_coeff * (forces - self.last_forces)
        self.last_forces = forces

        payload_mass = self.forces[2] / self.GRAVITATIONAL_ACCELERATION

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.PAYLOAD_DETECTION_THRESHOLD

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        self.avg_mass = np.mean(self.masses)
        self.var_mass = np.var(self.masses)

        self.payload_pub.publish(payload)


if __name__ == "__main__":
    payload_estimator = PayloadEstimator()
    rospy.spin()
