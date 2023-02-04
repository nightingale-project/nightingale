cmake_minimum_required(VERSION 3.0.2)
project(nightingale_bringup)

find_package(catkin REQUIRED COMPONENTS
  nightingale_dispatcher
  nightingale_msgs
  nightingale_navigation
  nightingale_perception
  nightingale_ros_bridge
  nightingale_utils
)

catkin_package(
)

install(FILES
  launch/nightingale_system.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

