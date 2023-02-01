#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from sensor_msgs.msg import JointState
from nightingale_msgs.msg import Payload


class PayloadEstimator:
    GRAVITY = 9.8

    def __init__(self):
        rospy.init_node("payload_estimator_node")

        self.forces = np.zeros((6, 1))
        self.last_forces = np.zeros((6, 1))
        self.filter_coeff = 0.5

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Replace with rosparam
        self.ARM_SIDE = "right"
        self.ARM_MASS = 2.5  # [kg]
        self.PAYLOAD_DETECTED_THRESHOLD = 0.1  # [kg]

        self.joint_state_sub = self.Subscriber(
            f"/movo/{self.ARM_SIDE}_arm/joint_states", JointState, self.joint_state_cb
        )

        self.payload_pub = self.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )
        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.dof = 7
        self.joint_link_suffixes = [
            f"{self.ARM_SIDE}_{link}"
            for link in [
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
        ]

    def joint_state_cb(self, msg):
        jacobian = self.try_get_jacobian()
        if jacobian is None:
            return

        forces = jacobian.T @ msg.effort

        self.forces = self.last_forces + self.filter_coeff * (forces - self.last_forces)
        self.last_forces = forces

        payload_mass = self.forces[2, 0] / self.GRAVITY - self.ARM_MASS

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.PAYLOAD_DETECTED_THRESHOLD

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        self.avg_mass = np.mean(self.masses)
        self.var_mass = np.var(self.masses)

        self.payload_pub.pub(payload)

    def try_get_jacobian(self):
        jacobian = np.empty((6, self.dof))

        for idx in range(self.dof):
            from_link = self.joint_link_suffixes[0]
            to_link = self.joint_link_suffixes[idx + 1]
            try:
                link_transform = self.tf_buffer.lookup_transform(
                    from_link, to_link, rospy.Time()
                )
                rot_axis = tf_conversions.transformations.quaternion_matrix(
                    link_transform.transform.rotation
                )[:, 2]
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.logerr(f"Arm transform lookup failed")
                return None

            jacobian[0:3, idx] = np.cross(
                rot_axis, link_transform.transform.translation
            )
            jacobian[3:6, idx] = rot_axis
        return jacobian


if __name__ == "__main__":
    payload_estimator = PayloadEstimator()
    rospy.spin()
