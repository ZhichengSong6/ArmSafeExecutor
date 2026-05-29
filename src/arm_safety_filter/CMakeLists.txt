cmake_minimum_required(VERSION 3.0.2)
project(arm_safety_filter)

add_compile_options(-std=c++14)

# ================================================================
# Catkin / ROS dependencies
# ================================================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  arm_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  message_filters
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_ros
  pcl_conversions
  urdf
)

# ================================================================
# PCL / OctoMap / HPP-FCL
# ================================================================
find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

# HPP-FCL exports its OctoMap compile definitions through pkg-config.
# These definitions must be propagated to the executable including
# <hpp/fcl/octree.h>.
find_package(PkgConfig REQUIRED)
pkg_check_modules(HPP_FCL REQUIRED hpp-fcl)

# ================================================================
# Catkin package export
# ================================================================
catkin_package(
  CATKIN_DEPENDS
    roscpp
    arm_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    message_filters
    tf2
    tf2_ros
    tf2_sensor_msgs
    pcl_ros
    pcl_conversions
    urdf
)

# ================================================================
# Include / link paths
# ================================================================
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${HPP_FCL_INCLUDE_DIRS}
)

link_directories(
  ${HPP_FCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# ================================================================
# Main safety-filter node
#
# Current functionality:
#   - command pass-through
#   - two-ToF NaN removal + per-sensor VoxelGrid + fusion
#   - subscription to robot_body_filter output
#   - link4 primitive geometry versus current-frame OcTree FCL distance
#   - latest-distance cache for the later Jacobian/QP step
#
# No command correction is active yet.
# ================================================================
add_executable(pointcloud_qp_safety_filter_node
  src/pointcloud_qp_safety_filter_node.cpp
)

add_dependencies(pointcloud_qp_safety_filter_node
  ${catkin_EXPORTED_TARGETS}
)

target_compile_options(pointcloud_qp_safety_filter_node PRIVATE
  ${HPP_FCL_CFLAGS_OTHER}
)

target_link_libraries(pointcloud_qp_safety_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${HPP_FCL_LIBRARIES}
)

# The earlier standalone link4 primitive distance executable is intentionally
# not built in the formal chain, because its distance computation is now
# merged into pointcloud_qp_safety_filter_node.