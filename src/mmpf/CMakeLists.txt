cmake_minimum_required(VERSION 2.8.3)
project(mmpf)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
  actionlib_msgs
  actionlib
  cartographer_ros_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package( 
CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mmpf_node src/mmpf_node.cpp)
target_link_libraries(mmpf_node ${catkin_LIBRARIES})
