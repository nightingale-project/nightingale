#!/usr/bin/env python3

import rospy
import tf2_ros
import tf_conversions
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from urdf_parser_py.urdf import Robot


class CollisionDetector:
    GRAVITATIONAL_ACCELERATION = 9.8  # [m/sec^2]

    def __init__(self, arm_side, box_mass, collision_detection_threshold):
        rospy.init_node("collision_detection_node")

        self.robot = Robot.from_parameter_server()

        self.dof = 7
        self.arm_side = arm_side
        self.box_mass = box_mass
        self.collision_detection_threshold = collision_detection_threshold
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
            "right": np.array(
                [
                    [-1.62545946, 8.78906103],
                    [-0.66488735, -9.69140664],
                    [-2.96047262, 0.98921177],
                    [-63.26756862, 8.20346974],
                    [-7.44528914, -4.88434097],
                    [-13.7096366, 17.69099921],
                    [48.5912395, 9.22901083],
                ]
            ),
            "left": np.array(
                [
                    [-2.49563151e00, -2.37168759e00],
                    [-1.78462115e00, 3.66522204e00],
                    [-3.41754670e00, -3.75087480e00],
                    [-3.91916221e00, 1.68547519e00],
                    [-3.03531958e00, -6.43691679e00],
                    [-7.22451467e00, -3.32532619e-01],
                    [3.77542114e-14, -2.40669758e-14],
                ]
            ),
        }
        self.error_models = {
            "right": np.array(
                [
                    [
                        1.95109362e01,
                        1.44005817e01,
                        -1.45924385e00,
                        -8.35374824e-15,
                        -2.96401194e-01,
                        -2.35777284e-01,
                        2.25790324e-15,
                    ],
                    [
                        1.44005817e01,
                        2.24846082e01,
                        3.66515516e00,
                        -2.27428444e-14,
                        -1.66168338e00,
                        -2.44940359e-01,
                        -1.40715896e-15,
                    ],
                    [
                        -1.45924385e00,
                        2.66515516e00,
                        4.32929229e00,
                        -5.92178800e-15,
                        -4.96042903e-01,
                        -6.25821647e-02,
                        3.92618002e-15,
                    ],
                    [
                        -8.35374824e-15,
                        -2.27428444e-14,
                        -5.92178800e-15,
                        1.94283891e-29,
                        1.43947963e-15,
                        2.28480868e-16,
                        -1.58258012e-30,
                    ],
                    [
                        -2.96401194e-01,
                        -1.66168338e00,
                        -4.96042903e-01,
                        1.43947963e-15,
                        1.50992237e-01,
                        3.26223572e-02,
                        2.44212239e-16,
                    ],
                    [
                        -2.35777284e-01,
                        -2.44940359e-01,
                        -6.25821647e-02,
                        2.28480868e-16,
                        3.26223572e-02,
                        2.11889851e-02,
                        -1.77455405e-17,
                    ],
                    [
                        2.25790324e-15,
                        -1.40715896e-15,
                        3.92618002e-15,
                        -1.58258012e-30,
                        2.44212239e-16,
                        -1.77455405e-17,
                        2.50033005e-29,
                    ],
                ]
            ),
            "left": np.array(
                [
                    [
                        9.88055768e01,
                        2.41331141e00,
                        7.07498122e00,
                        6.23710774e00,
                        7.46696889e00,
                        1.33010332e00,
                        2.52487445e-14,
                    ],
                    [
                        2.41331141e00,
                        5.28869064e00,
                        -3.48079620e00,
                        -2.29807168e00,
                        -2.74507190e00,
                        4.08223577e-01,
                        -3.99456509e-17,
                    ],
                    [
                        7.07498122e00,
                        -3.48079620e00,
                        3.57343121e00,
                        2.56552069e00,
                        3.31370422e00,
                        -2.80352452e-01,
                        2.68863201e-15,
                    ],
                    [
                        6.23710774e00,
                        -2.29807168e00,
                        2.56552069e00,
                        2.03109153e00,
                        2.61665944e00,
                        -1.59296845e-01,
                        2.59215439e-15,
                    ],
                    [
                        7.46696889e00,
                        -2.74507190e00,
                        3.31370422e00,
                        2.61665944e00,
                        3.51620637e00,
                        -2.43736384e-01,
                        3.06448851e-15,
                    ],
                    [
                        1.33010332e00,
                        4.08223577e-01,
                        -2.80352452e-01,
                        -1.59296845e-01,
                        -2.43736384e-01,
                        8.13145995e-02,
                        1.48376783e-16,
                    ],
                    [
                        2.52487445e-14,
                        -3.99456509e-17,
                        2.68863201e-15,
                        2.59215439e-15,
                        3.06448851e-15,
                        1.48376783e-16,
                        1.42352356e-29,
                    ],
                ]
            ),
        }

        self.collision_state = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.lookup_gravity_tf()

        self.collision_pub = rospy.Publisher(
            f"/nightingale/{self.arm_side}_arm/collision", Bool, queue_size=10
        )

        self.computed_torques_pub = rospy.Publisher(
            f"/nightingale/{self.arm_side}_arm/computed_torques",
            JointState,
            queue_size=10,
        )

        self.iter = 0
        self.iter_decimation = 10

        self.joint_state_sub = rospy.Subscriber(
            f"/movo/{self.arm_side}_arm/joint_states", JointState, self.joint_state_cb
        )

        self.collision_threshold_sub = rospy.Subscriber(
            f"/nightingale/{self.arm_side}_arm/collision_threshold",
            Float64,
            self.collision_threshold_cb,
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
        self.raw_measured_torques = np.array(torques)
        self.measured_torques = self.scale_torques(torques)
        self.computed_torques = self.jacobian.T @ self.gravity_forces

        # Replace right elbow measurement because the sensor is broken
        if self.arm_side == "right":
            self.measured_torques[3] = self.computed_torques[3]

    def detect_collisions(self):
        error = (self.measured_torques - self.computed_torques).reshape((self.dof, 1))
        error_distance = np.sqrt(
            error[:6, 0].T @ self.error_models[self.arm_side][:6, :6] @ error[:6, 0]
        )

        collision_detected = np.any(error_distance > self.collision_detection_threshold)
        if collision_detected:
            rospy.loginfo(f"Collision detected {error_distance}")
        if self.collision_state != collision_detected:
            self.collision_pub.publish(Bool(collision_detected))
            self.collision_state = not self.collision_state

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

            if idx == self.dof:
                gravity_forces[:3] += (
                    link_mass * self.GRAVITATIONAL_ACCELERATION * self.gravity_direction
                )
            else:
                ee_translation = (
                    self.translations[self.dof, :] - self.translations[idx, :]
                )
                gravity_forces[3:] += (
                    link_mass
                    * self.GRAVITATIONAL_ACCELERATION
                    * np.cross(ee_translation, self.gravity_direction)
                )

        self.gravity_forces = gravity_forces

    def scale_torques(self, torques):
        return (
            self.weights[self.arm_side][:, 0] * np.array(torques)
            + self.weights[self.arm_side][:, 1]
        )

    def pub_torques(self):
        joint_states = JointState()
        joint_states.name = [f"raw_measured_joint_{idx}" for idx in range(self.dof)] + [
            f"computed_joint_{idx}" for idx in range(self.dof)
        ]
        joint_states.effort = (
            self.raw_measured_torques.tolist() + self.computed_torques.tolist()
        )
        self.computed_torques_pub.publish(joint_states)

    def collision_threshold_cb(self, msg):
        self.collision_detection_threshold = msg.data


if __name__ == "__main__":
    collision_detector = CollisionDetector(
        "right", box_mass=1.0, collision_detection_threshold=55
    )
    collision_detector = CollisionDetector(
        "left", box_mass=0.0, collision_detection_threshold=50
    )
    rospy.spin()
