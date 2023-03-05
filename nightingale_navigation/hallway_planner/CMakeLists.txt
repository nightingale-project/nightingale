cmake_minimum_required(VERSION 3.0.2)
project(hallway_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  geometry_msgs
  nav_core
  global_planner
  pluginlib
  roscpp
  tf2_geometry_msgs
  tf2_ros
  cmake_modules
)

find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hallway_planner
  CATKIN_DEPENDS costmap_2d dynamic_reconfigure geometry_msgs nav_core pluginlib roscpp tf2_geometry_msgs tf2_ros
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hallway_planner.cpp
  src/geometry_primitives.cpp
  src/hallway_locator.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE _DEBUG_MAPS=true)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(test_hallway_locator test/test_hallway_locator.cpp)
target_link_libraries(test_hallway_locator
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PROJECT_NAME}
)
