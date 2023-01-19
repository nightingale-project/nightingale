#!/usr/bin/env python

import rospy

from nightingale_dispatcher.subtask import Subtask

class NavigateSubtask(Subtask):
    def __init__(self, name, room, feature, priority):
        super.__init__(name, priority)

        self.room = room
        self.feature = featurm

        # TODO Consider making a Singleton or share resources
        rospy.wait_for_service("/lookup_room_pose")
        self.lookup_proxy = rospy.ServiceProxy("/lookup_room_pose", RoomLookup)

    def execute(self):
        try:
            res = self.lookup_proxy(self.room, self.feature)
            
            if res.status == RoomPoseLookupResponse.ROOM_NOT_FOUND:
                return self.ARGUMENT_ERROR
            elif res.status == RoomPoseLookupResponse.FEATURE_NOT_FOUND:
                return self.ARGUMENT_ERROR
            return self.SUCCESS

        except rospy.ServiceException as e:
            print(f"Room Lookup service call failed: {e}")
            return self.SERVICE_ERROR
