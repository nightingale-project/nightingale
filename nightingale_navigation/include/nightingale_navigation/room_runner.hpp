#pragma once
#include <filesystem>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

namespace nightingale_navigation
{

/*
This class exposes nightingale specific actions: room_run and go_home
This class also optionally saves the robot pose in map frame
to disk in order to preserve it across boots
The file has the form:
x y yaw unix_time
*/
class RoomRunner
{
public:
    RoomRunner();
    ~RoomRunner();

    // No copies
    RoomRunner(const RoomRunner &) = delete;
    RoomRunner &operator=(const RoomRunner &) = delete;

    // No moves
    RoomRunner(RoomRunner &&) = delete;
    RoomRunner &operator=(RoomRunner &&) = delete;

    void run() const;

private:
    void initialize_pose();

    ros::NodeHandle nh_;
    bool persist_pose_;
    std::filesystem::path pose_path_;

    static constexpr auto EXPECTED_LINE_COUNT = 1;
    static constexpr auto INITIAL_POSE_TOPIC = "initialpose";
    // default cov matrix RVIZ uses when proving initial pose
    static constexpr std::array<double,36> INITIAL_POSE_COV = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
    static constexpr auto GLOBAL_FRAME= "/map";

};
}

