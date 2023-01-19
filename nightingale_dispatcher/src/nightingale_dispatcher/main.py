#!/usr/bin/env python

import rospy

import queue
from nightingale_msgs.srv import AddTaskDispatcher, CancelTaskDispatcher
from nightingale_dispatcher.subtask import Subtask
from nightingale_dispatcher.navigate_subtask import NavigateSubtask


class DispatcherNode:
    def __init__(self):
        rospy.init_node("dispatcher_node")

        self.tasks = []

        self.add_task_service = rospy.Service(
            "add_task", AddTaskDispatcher, self.handle_add_task
        )
        self.cancel_task_service = rospy.Service(
            "cancel_task", CancelTaskDispatcher, self.handle_cancel_task
        )

    def run(self):
        while not rospy.is_shutdown():
            task = self.tasks.get_no_wait()
            task.execute()

            rospy.spin_once()

    def handle_add_task(self, req):
        self.tasks.append({
            "name": req.name,
            "priority": None, # TODO determine rule to set int priority of patient
            "subtasks": [
            # TODO Populate with basic structure of a delivery
            # Nav -> Bed idle -> Nav -> Stock -> Nav -> Bed deliver -> Nav -> Idle
        ]})
        return True

    def handle_cancel_task(self, name):
        self.tasks = [task for task in self.tasks if self.tasks["name"] != req.name]
        return True


def main():
    dispatcher = DispatcherNode()

    rospy.spin()


if __name__ == "__main__":
    main()
