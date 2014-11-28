cmake_minimum_required(VERSION 2.8.3)
project(suturo_planning_executive)

find_package(catkin REQUIRED
            roslisp)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
)

###########
## Build ##
###########

# Deactivated for now, as the executable generation is not catkin-ready
# add_lisp_executable(suturo_planning_executive suturo-planning-executive suturo-planning-executive:main)

#############
## Install ##
#############

#############
## Testing ##
#############

