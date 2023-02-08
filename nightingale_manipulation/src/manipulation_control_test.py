#!/usr/bin/env python

import rospy

from nightingale_manipulation.manipulation_action_client import ManipulationControl


def main(*args, **kwargs):
    rospy.init_node("manipulation_control_test_node")
    man_ctrl = ManipulationControl()

    for i in range(3):
        rospy.loginfo("Going home")
        man_ctrl.jnt_ctrl.home()
        rospy.loginfo("Arrived home")

        rospy.sleep(3)

        rospy.loginfo("Going extended")
        man_ctrl.extend_handoff()
        rospy.loginfo("Arrived handoff")

        rospy.sleep(3)


if __name__ == "__main__":
    main()
