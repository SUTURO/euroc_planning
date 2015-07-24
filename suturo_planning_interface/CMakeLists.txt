cmake_minimum_required(VERSION 2.8.3)
project(suturo_planning_interface)

find_package(catkin REQUIRED COMPONENTS
  euroc_c2_msgs
  rospy
  std_msgs
  message_generation
  suturo_head_mover_msgs
)

catkin_package(CATKIN_DEPENDS rospy std_msgs message_runtime suturo_head_mover_msgs)
catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
