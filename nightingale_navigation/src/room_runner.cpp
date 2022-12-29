#include <nightingale_navigation/room_runner.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/console.h>
#include <numeric>
#include <fstream>

namespace nightingale_navigation
{
RoomRunner::RoomRunner() : nh_(), persist_pose_(false), pose_path_()
{
    nh_.param<bool>("persist_pose", persist_pose_, false);
    if (persist_pose_)
    {
        std::string path;
        nh_.param<std::string>("pose_path", path, "");
        pose_path_ = path;
        if (!pose_path_.is_absolute())
        {
            ROS_FATAL("Nightingale Room Runner must be provided absolute paths");
        }
        this->initialize_pose();
    }
    // init the 2 actions and the tf
}

RoomRunner::~RoomRunner() {}

void RoomRunner::initialize_pose()
{
    std::ifstream file(pose_path_);
    if (!file.is_open())
    {
        ROS_WARN("Cached Pose Path provided to Nightingale Room Runner does not exist");
        return;
    }
    /* File has the following form
       x y yaw unix_time
    */
    const auto line_count = std::count(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>(), '\n');
    double x = 0, y = 0, yaw = 0;
    size_t unix_time = std::numeric_limits<size_t>::max();
    if (line_count != EXPECTED_LINE_COUNT || !(file >> x >> y >> yaw >> unix_time) || unix_time > std::time(nullptr))
    {
        ROS_FATAL_STREAM("Nightingale RoomRunner cached file is corrupted! Please delete: " << pose_path_.string());
    }
    ROS_INFO_STREAM("Nightingale RoomRunner read cached pose successfully from " << pose_path_.string() << " {x:" << x << " y:" << y << " yaw: " << yaw << "}");
    // publish the pose
    [&]() -> void
    {
        ros::Publisher initial_pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(INITIAL_POSE_TOPIC, 1);
        // should wait til amcl listens to the topic before publishing to it
        // shouldnt be needed bc launch file uses delayed launch
        constexpr auto FAST_RATE = 100;
        for (ros::Rate r(FAST_RATE); initial_pose_pub.getNumSubscribers() == 0;)
        {
            ROS_INFO_THROTTLE(1,"Nightingale RoomRunner waiting for amcl to listen for initialpose");
            r.sleep();
        }
        auto msg = geometry_msgs::PoseWithCovarianceStamped();
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = GLOBAL_FRAME;
        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.position.z = 0;
        auto quat = tf2::Quaternion();
        quat.setRPY(0,0,yaw); 
        quat.normalize();
        msg.pose.pose.orientation = tf2::toMsg(quat);
        static_assert(msg.pose.covariance.size() == INITIAL_POSE_COV.size());
        std::copy(INITIAL_POSE_COV.cbegin(),INITIAL_POSE_COV.cend(),msg.pose.covariance.begin());
        ROS_ASSERT(memcmp(INITIAL_POSE_COV.data(),msg.pose.covariance.data(),INITIAL_POSE_COV.size()) == 0);
        initial_pose_pub.publish(msg);
        ROS_INFO("Nightingale RoomRunner published pose");
    }();
}

void RoomRunner::run() const {}

}

