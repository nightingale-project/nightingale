#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np
import matplotlib.pyplot as plt

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nightingale_msgs.msg import Payload


class PayloadEstimator:
    GRAVITY = 9.8  # [m/sec^2]
    PAYLOAD_DETECTION_THRESHOLD = 1  # [kg]

    def __init__(self):
        self.dof = 7
        self.ref_link_name = "base_link"
        self.joint_link_names = [
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

        self.forces = np.zeros(6)
        self.last_forces = np.zeros(6)
        self.filter_coeff = 0.5

        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.payload_pub = rospy.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )

        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_cb
        )

        self.analysis_service = rospy.Subscriber("analyze_arm_forces", Bool, self.analyze_arm_forces)
        self.analysis_num = 100
        self.analysis_dec = 10
        self.analysis_idx = 0
        self.analysis_forces = np.zeros(6, 100)

    def analyze_arm_forces(self, msg):
        mean = np.mean(self.analysis_forces, axis=1)
        covariance = np.cov((self.analysis_forces.T - mean).T)

        evals, evecs = np.linalg.eig(covariance)
        rospy.loginfo(mean)
        rospy.loginfo(covariance)

        if msg.data:
            labels = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"]
            for idx, label in zip(self.analysis_forces.shape[0], labels):
                plt.hist(self.analysis_forces[idx])
                plt.set_title(label)
            plt.show()

    def joint_state_cb(self, msg):
        jacobian, arm_forces = self.try_get_jacobian("right")
        if jacobian is None:
            return

        # 4: right_shoulder_pan_joint
        # 5: right_shoulder_lift_joint
        # 6: right_arm_half_joint
        # 7: right_elbow_joint
        # 8: right_wrist_spherical_1_joint
        # 9: right_wrist_spherical_2_joint
        # 10: right_wrist_3_joint

        forces = np.linalg.pinv(jacobian.T) @ msg.effort[4:11] - arm_forces

        self.forces = self.last_forces + self.filter_coeff * (forces - self.last_forces)
        self.last_forces = forces

        if self.analysis_idx % self.analysis_dec == 0:
            self.analysis_forces[:, (self.analysis_idx // self.analysis_dec) % self.analysis_num] = self.forces
        self.analysis_idx += 1

        payload_mass = self.forces[2] / self.GRAVITY

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.PAYLOAD_DETECTED_THRESHOLD

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        self.avg_mass = np.mean(self.masses)
        self.var_mass = np.var(self.masses)

        self.payload_pub.publish(payload)

    def try_get_jacobian(self, arm_side):
        arm_forces = np.zeros(6)
        jacobian = np.zeros((6, self.dof))

        for idx in range(self.dof):
            from_link = self.ref_link_name
            to_link = f"{arm_side}_{self.joint_link_suffixes[idx]}"
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
                rot_axis = rot_mat[:3, 2]

                transl = np.array(
                    [
                        link_transform.transform.translation.x,
                        link_transform.transform.translation.y,
                        link_transform.transform.translation.z,
                    ]
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.logerr(f"Arm transform lookup failed")
                return None, None

            arm_forces[:3] += np.array(
                [0, 0, -self.robot.link_map[to_link].inertial.mass * self.GRAVITY]
            ).T
            arm_forces[3:] += (
                self.robot.link_map[to_link].inertial.mass
                * self.GRAVITY
                * np.array([-transl[1], transl[0], 0]).T
            )

            jacobian[:3, idx] = np.cross(rot_axis, transl)
            jacobian[3:6, idx] = rot_axis
        return jacobian, arm_forces


if __name__ == "__main__":
    payload_estimator = PayloadEstimator()
    rospy.spin()
