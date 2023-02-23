#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Pose
from nightingale_msgs.srv import (
    RoomPoseLookup,
    RoomPoseLookupResponse,
    RobotConfigurationLookup,
    RobotConfigurationLookupResponse,
)


class RoomPoseService:
    def __init__(self):
        self.rooms = self.load_rooms()

        self.server = rospy.Service(
            "/nightingale/room_pose_lookup", RoomPoseLookup, self.room_pose_lookup
        )

    def load_rooms(self):
        rooms = {}

        rooms_params = rospy.get_param("/nightingale_utils/rooms")

        for room in rooms_params.keys():
            features = {}

            for feature in rooms_params[room].keys():
                pose = Pose()
                pose.position.x = rooms_params[room][feature]["position"]["x"]
                pose.position.y = rooms_params[room][feature]["position"]["y"]
                pose.position.z = rooms_params[room][feature]["position"]["z"]

                pose.orientation.x = rooms_params[room][feature]["orientation"]["x"]
                pose.orientation.y = rooms_params[room][feature]["orientation"]["y"]
                pose.orientation.z = rooms_params[room][feature]["orientation"]["z"]
                pose.orientation.w = rooms_params[room][feature]["orientation"]["w"]

                features[feature] = pose

            rooms[room] = features
        return rooms

    def room_pose_lookup(self, req):
        res = RoomPoseLookupResponse()
        if req.room not in self.rooms.keys():
            res.status = RoomPoseLookupResponse.ROOM_NOT_FOUND
            res.pose = Pose()
            return res
        elif req.feature not in self.rooms[req.room].keys():
            res.status = RoomPoseLookupResponse.FEATURE_NOT_FOUND
            res.pose = Pose()
            return res

        res.status = RoomPoseLookupResponse.SUCCESS
        res.pose = self.rooms[req.room][req.feature]
        return res


class RobotConfigurationService:
    def __init__(self):
        self.ss = rospy.Service(
            "/nightingale/robot_configuration_lookup",
            RobotConfigurationLookup,
            self.robot_configuration_lookup,
        )
        self.configurations = rospy.get_param("/nightingale_utils/joint_configurations")

    def robot_configuration_lookup(self, req):
        ret = RobotConfigurationLookupResponse()
        if req.lookup_cart == True:
            ret.cart_state.header.stamp = rospy.Time.now()
            body_part_string = {
                req.LEFT_ARM: "left_arm",
                req.RIGHT_ARM: "right_arm",
                req.TORSO: "torso",
                req.HEAD: "head",
            }[req.body_part]

            if req.configuration not in self.configurations.keys():
                return None
            configuration = self.configurations[req.configuration]

            if body_part_string not in configuration.keys():
                return None

            if req.joint_name not in configuration[body_part_string]["names"]:
                return None

            joint_idx = configuration[body_part_string]["names"].index(req.joint_name)

            ret.cart_state.name = configuration[body_part_string]["names"]

            # point
            ret.cart_state.header.frame_id = "base_link"
            ret.cart_state.pose.position.x = configuration[body_part_string][
                "cartesian"
            ][joint_idx][0]
            ret.cart_state.pose.position.y = configuration[body_part_string][
                "cartesian"
            ][joint_idx][1]
            ret.cart_state.pose.position.z = configuration[body_part_string][
                "cartesian"
            ][joint_idx][2]

            # orientation
            ret.cart_state.pose.orientation.x = configuration[body_part_string][
                "cartesian"
            ][joint_idx][3]
            ret.cart_state.pose.orientation.y = configuration[body_part_string][
                "cartesian"
            ][joint_idx][4]
            ret.cart_state.pose.orientation.z = configuration[body_part_string][
                "cartesian"
            ][joint_idx][5]
            ret.cart_state.pose.orientation.w = configuration[body_part_string][
                "cartesian"
            ][joint_idx][6]

            return ret
        else:
            ret.jnt_state.header.stamp = rospy.Time.now()
            body_part_string = {
                req.LEFT_ARM: "left_arm",
                req.RIGHT_ARM: "right_arm",
                req.TORSO: "torso",
                req.HEAD: "head",
            }[req.body_part]

            if req.configuration not in self.configurations.keys():
                return None
            configuration = self.configurations[req.configuration]

            if body_part_string not in configuration.keys():
                return None

            ret.jnt_state.name = configuration[body_part_string]["names"]
            ret.jnt_state.position = configuration[body_part_string]["joints"]
            ret.jnt_state.velocity = [0.0 for jnt in ret.jnt_state.name]
            return ret


def main():
    rospy.init_node("nightingale_utils_node")

    rps = RoomPoseService()
    rcs = RobotConfigurationService()

    rospy.spin()


if __name__ == "__main__":
    main()
