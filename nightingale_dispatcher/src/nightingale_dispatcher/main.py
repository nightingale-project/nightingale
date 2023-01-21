#!/usr/bin/env python

import rospy
import actionlib

import queue
from actionlib.simple_action_client import SimpleGoalState
from nightingale_msgs.msg import TaskAction, TaskGoal
from nightingale_msgs.srv import AddTaskDispatcher

class DispatcherNode:
    def __init__(self):
        rospy.init_node("dispatcher_node")

        self.task_client = actionlib.SimpleActionClient("task", TaskAction)
        self.task_client.wait_for_server()

        self.tasks = queue.Queue()

        self.add_task_service = rospy.Service(
            "add_task", AddTaskDispatcher, self.handle_add_task
        )

    def handle_add_task(self, req):
        goal = TaskGoal()
        goal.name = req.name
        # goal.priority = 0 # TODO determine rule to set int priority of patient
        goal.items_json = req.items_json

        self.tasks.put(goal)
        if self.task_client.simple_state == SimpleGoalState.DONE:
            # No goal in progress, execute goal now
            self.task_client.send_goal(self.tasks.get(), self.handle_done_task)

        return True

    def handle_done_task(self, status, result):
        if not self.tasks.empty():
            goal = self.tasks.get()
            self.task_client.send_goal(goal, self.handle_done_task)

def main():
    dispatcher = DispatcherNode()

    rospy.spin()


if __name__ == "__main__":
    main()
