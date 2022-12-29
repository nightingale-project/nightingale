#include <ros/ros.h>
#include <ros/console.h>
#include <nightingale_navigation/room_runner.hpp>

int main(int argc, char **argv)
{
    using namespace nightingale_navigation;
    ros::init(argc, argv, "nightingale_room_runner");
    ROS_INFO("Initializing the nightingale_room_runner node");
    auto rr = RoomRunner();
    rr.run();
    return 0;
}

