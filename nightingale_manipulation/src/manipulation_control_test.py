#!/usr/bin/env python

import rospy

from nightingale_manipulation.manipulation_action_client import ManipulationControl


def main(*args, **kwargs):
    rospy.init_node("manipulation_control_test_node")
    man_ctrl = ManipulationControl()

    for i in range(3):
        #man_ctrl.jnt_ctrl.home()
        man_ctrl.jnt_ctrl.cmd_right_arm(man_ctrl.jnt_ctrl.right_arm_home_joint_values, blocking=False)
        man_ctrl.jnt_ctrl.right_arm.wait_for_result()
        
        rospy.sleep(3)

        man_ctrl.extend_handoff()
        #man_ctrl.gpr_ctrl.cmd_right_gripper(man_ctrl.gpr_ctrl.right_open_goal)

        #rospy.sleep(1)
        #man_ctrl.gpr_ctrl.cmd_right_gripper(man_ctrl.gpr_ctrl.right_closed_goal)

if __name__ == "__main__":
    main()


