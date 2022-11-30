import rospy
from movo_msgs.msg import ConfigCmd
from move_base_action_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header

class MoveBaseEvaluation:
    def __init__(self):
        self._cfg_pub = rospy.Publisher("/movo/gp_command", ConfigCmd, queue_size=10)
        self._move_goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)
        self._move_res_pub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_callback, queue_size=10)

        self.idx = 0

        self.generate_poses()

    def generate_poses(self, travel_dist=1.0, num_loops=10):
        ref_poses = [[2, 0, 0], [4, 0, -math.pi/2], [4, -2, -math.pi], [2, -2, -3*math.pi/2]]

        self.poses = [None for i in range(num_loops * len(ref_poses)):
        for i in range(num_loops * len(ref_poses)):
            n = i % len(ref_poses)

            tgt_pose = PoseStamped()
            tgt_pose.pose.position.x = ref_poses[n][0]
            tgt_pose.pose.position.y = ref_poses[n][1]
            tgt_pose.pose.orientation.w = math.cos(ref_poses[n][2] / 2)
            tgt_pose.pose.orientation.z = math.sin(ref_poses[n][2] / 2)
            tgt_pose.header.seq = i
            tgt_pose.header.stamp = rospy.get_rostime()
            tgt_pose.frame_id = "/map"

            self.poses[i] = tgt_pose

    def start_tractor_mode(self):
        cfg_cmd = ConfigCmd()
        cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_SET_OPERATIONAL_MODE'
        cfg_cmd.gp_param = TRACTOR_REQUEST
        cfg_cmd.header.stamp = rospy.get_rostime()
        
        self._cfg_pub.publish(self._cfg_cmd)
        rospy.sleep(0.1)

    def stop_tractor_mode(self):
        cfg_cmd = ConfigCmd()
        cfg_cmd.gp_cmd = 'GENERAL_PURPOSE_CMD_NONE'
        cfg_cmd.gp_param = 0
        cfg_cmd.header.stamp = rospy.get_rostime()
        
        self.cfg_pub.publish(self._cfg_cmd)
        rospy.sleep(0.1)

    def result_callback(self, msg):
        if self.idx < len(self.poses) - 1:
            self.idx += 1
            self._move_goal_pub(self.poses[self.idx])

    def run(self):
        self.start_tractor_mode()

        while self.idx < len(self.poses):
            rospy.spin_once()

        self.stop_tractor_mode()

if __name__ == "__main__":
    rospy.init_node("move_base_eval_node")

    move_base_eval = MoveBaseEvaluation()
    move_base_eval.run()

