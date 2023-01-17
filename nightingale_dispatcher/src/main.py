#!/usr/bin/env python

import rospy

import queue


class TaskManager:
    def __init__(self, maxsize=0):
        self.tasks = queue.PriorityQueue(maxsize)


class Task:
    SUCCESS = 0
    ARGUMENT_ERROR = 1
    SERVICE_ERROR = 2
    TOPIC_ERROR = 3
    ACTION_ERROR = 4

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
        res = self._try_lookup_room()
        if res != self.SUCCESS:
            return res

        

    def _try_lookup_room(self):
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


class DispatcherNode:
    def __init__(self):
        rospy.init_node("dispatcher_node")


def main():
    node = RoomPoseServiceNode()

    rospy.spin()


if __name__ == "__main__":
    main()
