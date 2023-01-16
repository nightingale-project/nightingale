#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from nightingale_dispatch.srv import RoomPoseLookup, RoomPoseLookupResponse

class RoomPoseServiceNode:
    def __init__(self):
        rospy.init_node("room_pose_service_node")

        self.rooms = self.load_rooms()

        self.server = rospy.Service("lookup_room_pose", RoomPoseLookup, self.lookup_room_pose)

    def load_rooms(self):
        rooms = {}

        rooms_params = rospy.get_param("rooms")

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

    def lookup_room_pose(self, req):
        res = RoomPoseLookupResponse()
        if req.room in self.rooms.keys():
            res.status = RoomPoseLookupResponse.ROOM_NOT_FOUND
            res.pose = Pose()
            return res
        elif req.feature in self.rooms[req.room].keys():
            res.status = RoomPoseLookupResponse.FEATURE_NOT_FOUND
            res.pose = Pose()
            return res

        res.status = RoomPoseLookupResponse.SUCCESS
        res.pose = self.rooms[req.room][req.feature]
        return res

def main():
    node = RoomPoseServiceNode()

    rospy.spin()

if __name__ == "__main__":
    main()
