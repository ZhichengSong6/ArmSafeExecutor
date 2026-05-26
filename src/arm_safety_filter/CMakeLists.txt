cmake_minimum_required(VERSION 3.0.2)
project(arm_safety_filter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  arm_msgs
  sensor_msgs
  geometry_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    arm_msgs
    sensor_msgs
    geometry_msgs
    message_filters
    tf2
    tf2_ros
    tf2_sensor_msgs
    pcl_ros
    pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

add_executable(pointcloud_qp_safety_filter_node
  src/pointcloud_qp_safety_filter_node.cpp
)

add_dependencies(pointcloud_qp_safety_filter_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pointcloud_qp_safety_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)