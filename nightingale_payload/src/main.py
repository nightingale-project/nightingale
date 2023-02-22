#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from nightingale_msgs.msg import Payload
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from urdf_parser_py.urdf import Robot


class PayloadEstimator:
    GRAVITATIONAL_ACCELERATION = -9.8  # [m/sec^2]
    PAYLOAD_DETECTION_THRESHOLD = 1  # [kg]

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
        self.gravity_direction = np.zeros(3)
        self.gravity_forces = np.zeros(6)

        self.forces = np.zeros(6)
        self.last_forces = np.zeros(6)
        self.filter_coeff = 1

        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.payload_pub = rospy.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lookup_gravity_tf()

        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_cb
        )

        self.analysis_sub = rospy.Subscriber(
            "analyze_arm_forces", Bool, self.analyze_arm_forces
        )
        self.analysis_num = 1000
        self.analysis_dec = 5
        self.analysis_idx = 0
        self.analysis_forces = np.zeros((6, self.analysis_num))

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

        self.analysis_idx += 1
        if self.analysis_idx % self.analysis_dec == 0:
            self.analysis_forces[
                :, (self.analysis_idx // self.analysis_dec) % self.analysis_num
            ] = self.forces
            if self.analysis_idx // self.analysis_dec == self.analysis_num:
                rospy.loginfo("Ready to analyze")
                self.analysis_idx = 0

    def analyze_arm_forces(self, msg):
        mean = np.mean(self.analysis_forces, axis=1)
        covariance = np.cov((self.analysis_forces.T - mean).T)

        evals, evecs = np.linalg.eig(covariance)
        principal_axes = np.sqrt(evals) * evecs

        timestamp = int(rospy.get_time())
        filename = f"/home/lyndon/Documents/mte_4812/arm_force/data/arm_force_analysis_{timestamp}.npz"
        np.savez(
            filename,
            forces=self.analysis_forces,
            gravity_direction=self.gravity_direction,
            mean=mean,
            covariance=covariance,
            evals=evals,
            evecs=evecs,
            principal_axes=principal_axes,
        )
        rospy.loginfo(f"Saved to {filename}")

        rospy.loginfo(mean)
        rospy.loginfo(covariance)

    def lookup_gravity_tf(self):
        try:
            base_link_transform = self.tf_buffer.lookup_transform(
                "base_link",
                f"{self.arm_side}_base_link",
                rospy.Time(0),
                rospy.Duration(20),
            )

            rot_mat = tf_conversions.transformations.quaternion_matrix(
                [
                    base_link_transform.transform.rotation.x,
                    base_link_transform.transform.rotation.y,
                    base_link_transform.transform.rotation.z,
                    base_link_transform.transform.rotation.w,
                ]
            )
            self.gravity_direction = rot_mat[:3, 2]
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
            to_link = f"{self.arm_side}_{self.joint_link_suffixes[idx]}"
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

    def compute_arm_forces(self, torques):
        forces = np.linalg.pinv(self.jacobian.T) @ torques - self.gravity_forces

        self.forces = self.last_forces + self.filter_coeff * (forces - self.last_forces)
        self.last_forces = forces

        payload_mass = (
            np.dot(self.forces[:3], -self.gravity_direction)
            / self.GRAVITATIONAL_ACCELERATION
        )

        rospy.loginfo(f"{np.sqrt(np.sum(np.square(self.forces[:3])))} {payload_mass}")

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.PAYLOAD_DETECTION_THRESHOLD

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        self.avg_mass = np.mean(self.masses)
        self.var_mass = np.var(self.masses)

        self.payload_pub.publish(payload)


if __name__ == "__main__":
    payload_estimator = PayloadEstimator("right")
    rospy.spin()
