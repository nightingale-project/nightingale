#!/usr/bin/env python3
import rospy

class Task:
    ERROR = -1
    SUCCESS = 0

    def execute(self):
        """
        execute function must be overriden by classes that inherit task
        :param: None
        :return: (Enum) status
        """
        name = type(self)
        rospy.logerr(str(name) + ".execute not implemented")
        raise NotImplementedError
