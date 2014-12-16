cmake_minimum_required(VERSION 2.8.3)
project(suturo_planning_plans)

find_package(catkin REQUIRED COMPONENTS
  euroc_c2_msgs
  rospy
  std_msgs
)

catkin_python_setup()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


