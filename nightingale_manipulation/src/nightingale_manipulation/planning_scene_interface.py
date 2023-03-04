from six.moves import input

import sys
import rospy
import moveit_commander
import geometry_msgs.msg


class PlanningSceneInterface(object):

    def __init__(self):
        super(PlanningSceneInterface, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "right_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % self.planning_frame)

        self.eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % self.eef_link)

        self.group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        self.box_in_world = False

    def __del__(self):
        self.remove_box()

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            upper = "right_upper_wall" in self.scene.get_known_object_names()
            lower = "right_lower_wall" in self.scene.get_known_object_names()
            top = "right_top_wall" in self.scene.get_known_object_names()
            if upper and lower and top:
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, timeout=4):
        if self.box_in_world:
            return False

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "upper_body_link"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = -0.43
        box_pose.pose.position.z = -0.15

        upper_box = geometry_msgs.msg.PoseStamped()
        upper_box.header.frame_id = "upper_body_link"
        upper_box.pose.orientation.w = 0.924
        upper_box.pose.orientation.x = -0.383
        upper_box.pose.position.x = 0.25
        upper_box.pose.position.y = -0.33
        upper_box.pose.position.z = 0.25

        top_box = geometry_msgs.msg.PoseStamped()
        top_box.header.frame_id = "upper_body_link"
        top_box.pose.orientation.w = 1.0
        top_box.pose.position.x = 0.32
        top_box.pose.position.y = -0.15
        top_box.pose.position.z = 0.36

        self.scene.add_box("right_lower_wall", box_pose, size=(0.5, 0.05, 0.6))
        self.scene.add_box("right_upper_wall", upper_box, size=(0.5, 0.05, 0.32))
        self.scene.add_box("right_top_wall", top_box, size=(0.35, 0.15, 0.03))

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            upper = "right_upper_wall" in self.scene.get_known_object_names()
            lower = "right_lower_wall" in self.scene.get_known_object_names()
            top = "right_top_wall" in self.scene.get_known_object_names()
            if upper and lower and top:
                self.box_in_world = True
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def remove_box(self, timeout=4):
        if not self.box_in_world:
            return True

        self.scene.remove_world_object("right_lower_wall")
        self.scene.remove_world_object("right_upper_wall")
        self.scene.remove_world_object("right_top_wall")
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            upper = "right_upper_wall" not in self.scene.get_known_object_names()
            lower = "right_lower_wall" not in self.scene.get_known_object_names()
            top = "right_top_wall" not in self.scene.get_known_object_names()
            if upper and lower and top:
                self.box_in_world = False
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False


def main():
    rospy.init_node("planning_scene_interface_demo")
    tutorial = PlanningSceneInterface()
    try:
        while True:
            input("============ Press `Enter` to add a box to the planning scene ...")
            tutorial.add_box()

            input(
                "============ Press `Enter` to remove the box from the planning scene ..."
            )
            tutorial.remove_box()
            rospy.sleep(4.0)

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        tutorial.remove_box()


if __name__ == "__main__":
    main()

