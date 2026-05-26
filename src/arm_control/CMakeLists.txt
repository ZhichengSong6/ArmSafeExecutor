cmake_minimum_required(VERSION 3.0.2)
project(arm_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  interactive_markers
  tf2
  tf2_ros
  tf2_geometry_msgs
  arm_msgs
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    geometry_msgs
    std_msgs
    visualization_msgs
    interactive_markers
    tf2
    tf2_ros
    tf2_geometry_msgs
    arm_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)

add_executable(pinocchio_controller_node
  src/pinocchio_controller_node.cpp
)

add_dependencies(pinocchio_controller_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pinocchio_controller_node
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
)

add_executable(interactive_marker_node
  src/interactive_marker.cpp
)

add_dependencies(interactive_marker_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(interactive_marker_node
  ${catkin_LIBRARIES}
)