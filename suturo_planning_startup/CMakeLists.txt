cmake_minimum_required(VERSION 2.8.3)
project(suturo_planning_startup)

find_package(catkin REQUIRED COMPONENTS
  euroc_c2_msgs
  rospy
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs suturo_planning_plans suturo_planning_task_selector
)