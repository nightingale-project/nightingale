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

        self.poses = self.generate_poses()

    def generate_poses(self, travel_dist=1.0, num_loops=10):
        for i in range(num_loops * 4):
            n = i % 4

            tgt_pose = PoseStamped()
            tgt_pose.pose.position.x = travel_dist * (((n & 2) >> 1) ^ (n & 1))
            tgt_pose.pose.position.y = travel_dist * ((n & 2) >> 1)
            tgt_pose.pose.orientation.w = math.cos(math.pi / 4 * n)
            tgt_pose.pose.orientation.z = math.sin(math.pi / 4 * n)
            tgt_pose.header.seq = i
            tgt_pose.header.stamp = rospy.get_rostime()
            tgt_pose.frame_id = "/map"

            self.poses.append(tgt_pose)

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

