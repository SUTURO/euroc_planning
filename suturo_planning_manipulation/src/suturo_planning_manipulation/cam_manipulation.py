#!/usr/bin/env python
import rospy
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *


__author__ = 'benny'


class CamManipulation(object):
    def __init__(self):
        rospy.init_node('cam_manipulation')
        rospy.wait_for_service('/euroc_interface_node/move_along_joint_path')
        self.__service = rospy.ServiceProxy('/euroc_interface_node/move_along_joint_path', MoveAlongJointPath)

    def tilt(self, radian):
        cartesian_limits = CartesianLimits()
        cartesian_limits.translational.max_velocity = 0
        cartesian_limits.translational.max_acceleration = 0
        cartesian_limits.rotational.max_velocity = 0
        cartesian_limits.rotational.max_acceleration = 0
        joint_limits = []
        limit = Limits()
        limit.max_velocity = 0.4
        limit.max_acceleration = 2.0
        joint_limits.append(limit)
        path = SearchIkSolutionResponse()
        path.solution.q = [radian]
        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)
        resp = self.__service(['cam_tilt'], [path.solution], ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.stop_reason

    def pan(self, radian):
        cartesian_limits = CartesianLimits()
        cartesian_limits.translational.max_velocity = 0
        cartesian_limits.translational.max_acceleration = 0
        cartesian_limits.rotational.max_velocity = 0
        cartesian_limits.rotational.max_acceleration = 0
        joint_limits = []
        limit = Limits()
        limit.max_velocity = 0.4
        limit.max_acceleration = 2.0
        joint_limits.append(limit)
        path = SearchIkSolutionResponse()
        path.solution.q = [radian]
        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)
        resp = self.__service(['cam_pan'], [path.solution], ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.stop_reason