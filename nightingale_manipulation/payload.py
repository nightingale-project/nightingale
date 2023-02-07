#!/usr/bin/env python3

import numpy as np
import rospy
import tf_conversions

from nightingale_msgs.msg import Payload
from urdf_parser_py.urdf import Robot


class Payload:
    GRAVITY = 9.8

    def __init__(self, arm_side, detection_threshold):
        self.arm_side = arm_side
        self.detection_threshold

        self.robot = Robot.from_parameter_server()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.arm_links = [
            f"{arm_side}_{link_name}"
            for link_name in [
                "base_link",
                "shoulder_link",
                "arm_half_1_link",
                "arm_half_2_link",
                "forearm_link",
                "wrist_spherical_1_link",
                "wrist_spherical_2_link",
                "wrist_3_link",
                "ee_link",
            ]
        ]
        self.link_indices = {
            link_name: idx for idx, link_name in enumerate(self.arm_links)
        }

        self.jacobian = np.zeros((6, 7))  # 6 Cartesion DOF x 7 Joint DOF
        self.forces = np.zeros(6)  # [Fx[N] Fy[N] Fz[N] Mx[Nm] My[Nm] Mz[Nm]]

        self.mass_num = 100
        self.mass_idx = 0
        self.masses = np.zeros(self.mass_num)

        self.payload_pub = self.Publisher(
            f"/nightingale/payload", Payload, queue_size=10
        )

    def run(self, link_transforms, joint_velocity, joint_effort):
        self._update(link_transforms)

        self.forces = np.linalg.pinv(self.jacobian.T) @ joint_effort - self.arm_forces
        # TODO Implement lowpass filter

        self._publish_payload()

    def _publish_payload(self):
        payload_mass = self.forces[2] / self.GRAVITY

        self.masses[self.mass_idx] = payload_mass
        self.mass_idx = (self.mass_idx + 1) % self.mass_num

        payload = Payload()
        payload.mass = payload_mass
        payload.detected = payload_mass > self.detection_threshold
        payload.avg_mass = np.mean(self.masses)
        payload.var_mass = np.var(self.masses)

        self.payload_pub.publish(payload)

    def _update(self, link_transforms):
        if any(
            link_name not in link_transforms.keys()
            or link_transforms[link_name] is None
            for link_name in self.arm_links
        ):
            return

        jacobian = np.zeros((6, 7))
        arm_forces = np.zeros(6)

        for link in enumerate(self.link_indices):
            orientation_quaternion = np.array(
                [
                    link_transforms[link].transform.rotation.x,
                    link_transforms[link].transform.rotation.y,
                    link_transforms[link].transform.rotation.z,
                    link_transforms[link].transform.rotation.w,
                ]
            )
            rot_mat = tf_conversions.transformations.quaternion_matrix(
                orientation_quaternion
            )
            rot_axis = rot_mat[:3, 2]

            translation = np.array(
                [
                    link_transforms[link].transform.translation.x,
                    link_transforms[link].transform.translation.y,
                    link_transforms[link].transform.translation.z,
                ]
            )

            arm_forces[:3] += np.array(
                [0, 0, -self.robot.link_map[link].inertial.mass * self.GRAVITY]
            ).T
            arm_forces[3:] += (
                self.robot.link_map[link].inertial.mass
                * self.GRAVITY
                * np.array([-translation[1], translation[0], 0]).T
            )

            jacobian[:3, self.link_indices[link]] = np.cross(rot_axis, translation)
            jacobian[3:6, self.link_indices[link]] = rot_axis

        self.jacobian = jacobian

        # TODO calculate coriolis forces
        # x_dot = J * q_dot => omega = x_dot[3:] => F = -2 * m * (omega x v)
        # TODO calculate centfigual forces
        # x_dot = J * q_dot => omega = x_dot[3:] => omega_dot = (omega_t - omega_t_1) / dt => F = -I * omega_dot
        self.arm_forces = arm_forces
