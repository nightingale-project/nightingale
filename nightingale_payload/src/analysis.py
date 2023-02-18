#!/usr/bin/env python

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from nightingale_msgs.msg import Payload
from urdf_parser_py.urdf import Robot


class PayloadEstimator:
    GRAVITY = 9.8  # [m/sec^2]
    PAYLOAD_DETECTION_THRESHOLD = 1  # [kg]

    def __init__(self):
        rospy.init_node("payload_estimator_node")

        self.robot = Robot.from_parameter_server()
        rospy.loginfo(f"{self.robot.link_map['right_shoulder_link'].inertial}")

        self.dof = 7
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

        self.analysis_service = rospy.Subscriber(
            "analyze_arm_forces", Bool, self.analyze_arm_forces
        )
        self.analysis_num = 100
        self.analysis_dec = 5
        self.analysis_idx = 0
        self.analysis_forces = np.zeros((6, self.analysis_num))

    def analyze_arm_forces(self, msg):
        mean = np.mean(self.analysis_forces, axis=1)
        covariance = np.cov((self.analysis_forces.T - mean).T)

        evals, evecs = np.linalg.eig(covariance)
        principal_axes = np.sqrt(evals) * evecs

        np.savez(
            f"/home/lyndon/Documents/mte_4812/arm_force_analysis.npz",
            self.analysis_forces,
            mean,
            covariance,
            evals,
            evecs,
            principal_axes,
        )

        rospy.loginfo(mean)
        rospy.loginfo(covariance)

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

        forces = np.linalg.pinv(jacobian.T) @ msg.effort[4:11]  # - arm_forces

        self.forces = self.last_forces + self.filter_coeff * (forces - self.last_forces)
        self.last_forces = forces

        if self.analysis_idx % self.analysis_dec == 0:
            self.analysis_forces[
                :, (self.analysis_idx // self.analysis_dec) % self.analysis_num
            ] = self.forces
            if self.analysis_idx // self.analysis_dec == self.analysis_num:
                rospy.loginfo("Ready to analyze")
        self.analysis_idx += 1

        payload_mass = self.forces[2] / self.GRAVITY

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.PAYLOAD_DETECTION_THRESHOLD
        payload.arm_forces = self.forces.tolist()

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        self.avg_mass = np.mean(self.masses)
        self.var_mass = np.var(self.masses)

        self.payload_pub.publish(payload)

    def try_get_jacobian(self, arm_side):
        arm_forces = np.zeros(6)
        jacobian = np.zeros((6, self.dof))

        for idx in range(self.dof):
            from_link = f"{arm_side}_{self.joint_link_suffixes[0]}"
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
                rospy.logerr(
                    f"Arm transform lookup from {from_link} to {to_link} failed"
                )
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

"""
mean=[ 1.48798556  4.02257134 36.51546116]
cov=[[ 0.00392623  0.00094428  0.00076409]
 [ 0.00094428  0.00468538 -0.00017132]
 [ 0.00076409 -0.00017132  0.00159611]]

mean=[ 0.85829068 26.44326372 31.79634967]
cov=[[ 2.60679306e-03 -9.14328342e-05  1.97105243e-05]
 [-9.14328342e-05  1.52748140e-03 -2.44626672e-04]
 [ 1.97105243e-05 -2.44626672e-04  1.51252473e-03]]
"""
