cmake_minimum_required(VERSION 3.0.2)
project(arm_sim_node)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  urdf
  geometry_msgs
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  tf2_geometry_msgs
  arm_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    urdf
    geometry_msgs
    visualization_msgs
    interactive_markers
    tf2
    tf2_ros
    tf2_geometry_msgs
    arm_msgs
    gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_sim_node
  src/arm_sim_node.cpp
)

add_dependencies(arm_sim_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(arm_sim_node
  ${catkin_LIBRARIES}
)