cmake_minimum_required(VERSION 2.8.3)
project(suturo_planning_perception)

find_package(catkin REQUIRED COMPONENTS
  euroc_c2_msgs
  rospy
  std_msgs
  smach
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


