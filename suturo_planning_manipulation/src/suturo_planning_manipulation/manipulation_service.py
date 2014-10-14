#!/usr/bin/env python

__author__ = 'bennypi'

import roslib; roslib.load_manifest('euroc_c2_demos')
import rospy
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from math import pi


class ManipulationService(object):
    def __init__(self):
        servicename = '/euroc_interface_node/move_along_joint_path'
        rospy.wait_for_service(servicename)
        self.__service = rospy.ServiceProxy(servicename, MoveAlongJointPath)

    def move(self, path):
        joint_limits = []

        cartesian_limits = CartesianLimits()
        cartesian_limits.translational.max_velocity = 0.165
        cartesian_limits.translational.max_acceleration = 4
        cartesian_limits.rotational.max_velocity = 10 * pi / 180.0
        cartesian_limits.rotational.max_acceleration = 100 * pi / 180.0

        for i in range(len(path.joint_trajectory.joint_names)):
            limit = Limits()
            joint_name = path.joint_trajectory.joint_names[i]
            if joint_name.startswith("lwr"):
                limit.max_velocity = 20 * pi / 180.0
                limit.max_acceleration = 400 * pi / 180.0
            if joint_name.startswith("axis"):
                limit.max_velocity = 0.165
                limit.max_acceleration = 4
            if joint_name.startswith("joint_before_finger1"):
                limit.max_velocity = 0.5
                limit.max_acceleration = 20
            joint_limits.append(limit)

        config = []
        if path.joint_trajectory.joint_names[0].startswith("joint_before_finger"):
            # rename jointname
            path.joint_trajectory.joint_names = ["gripper"]
            joint_limits.pop()
            for i in range(len(path.joint_trajectory.points)):
                blub = SearchIkSolutionResponse()
                blub.solution.q = [2 * path.joint_trajectory.points[i].positions[1]]
                config.append(blub.solution)
        else:
            for i in range(len(path.joint_trajectory.points)):
                blub = SearchIkSolutionResponse()
                blub.solution.q = path.joint_trajectory.points[i].positions
                config.append(blub.solution)

        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)

        print path.joint_trajectory.joint_names

        resp = self.__service(path.joint_trajectory.joint_names, config, ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise ManipulationServiceException(resp.error_message)
        return resp.stop_reason


class ManipulationServiceException(Exception):
    pass