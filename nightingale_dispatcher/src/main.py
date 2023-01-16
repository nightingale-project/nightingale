#!/usr/bin/env python

import rospy


class TaskManager:
    def __init__(self):
        self.tasks = {}


class Task:
    def __init__(self, name, priority=0):
        self.name = name
        self.priority = priority

    def execute(self):
        pass


class NavigateTask(Task):
    def __init__(self, name, room, feature, priority):
        super.__init__(name, priority)

        self.room = room
        self.feature = feature

        # Consider making a Singleton or share resources
        rospy.wait_for_service("/lookup_room_pose")
        self.lookup_proxy = rospy.ServiceProxy("/lookup_room_pose", RoomLookup)

    def execute(self):
        
    

    def _try_lookup_room(self):
        try:
            res = self.lookup_proxy(self.room, self.feature)
            
            if res.status == RoomPoseLookupResponse.FEATURE_NOT_FOUND

        except rospy.ServiceException as e:
            print(f"Room Lookup service call failed: {e}")
            return False


class DispatcherNode:
    def __init__(self):
        rospy.init_node("dispatcher_node")


def main():
    node = RoomPoseServiceNode()

    rospy.spin()


if __name__ == "__main__":
    main()
