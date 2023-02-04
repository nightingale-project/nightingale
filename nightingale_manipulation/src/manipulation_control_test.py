#!/usr/bin/env python

import rospy

from nightingale_manipulation.manipulation_action_client import ManipulationControl


def main(*args, **kwargs):
    rospy.init_node("manipulation_control_test_node")
    man_ctrl = ManipulationControl()

    for i in range(3):
        #man_ctrl.jnt_ctrl.home()
        rospy.loginfo("Going home")
        man_ctrl.jnt_ctrl.home()
#        man_ctrl.jnt_ctrl.cmd_right_arm(man_ctrl.jnt_ctrl.right_arm_home_joint_values)
        rospy.loginfo("Arrived home")        

        rospy.sleep(3)

        rospy.loginfo("Going extended")
        man_ctrl.extend_handoff()
        rospy.loginfo("Arrived handoff")

    #    rospy.loginfo("Going gripper open")
 #       man_ctrl.gpr_ctrl.cmd_right_gripper(man_ctrl.gpr_ctrl.right_open_goal)
     #   rospy.loginfo("Arrived gripper open")

   #     rospy.sleep(1)
      #  rospy.loginfo("Going gripper closed")
  #      man_ctrl.gpr_ctrl.cmd_right_gripper(man_ctrl.gpr_ctrl.right_closed_goal)
       # rospy.loginfo("Arrived gripper closed")

        rospy.sleep(3)

if __name__ == "__main__":
    main()


