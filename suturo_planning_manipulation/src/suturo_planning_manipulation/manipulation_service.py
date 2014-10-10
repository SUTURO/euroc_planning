#!/usr/bin/env python

__author__ = 'bennypi'

import roslib; roslib.load_manifest('euroc_c2_demos')
import rospy
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from trajectory_msgs import msg
import trajectory_msgs
from moveit_msgs.msg import RobotTrajectory
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from math import pi
import copy


class ManipulationService(object):
    def __init__(self):
        # nodename = 'ManipulationService'
        # rospy.init_node(nodename, anonymous=True)
        servicename = '/euroc_interface_node/move_along_joint_path'
        rospy.wait_for_service(servicename)
        self.__service = rospy.ServiceProxy(servicename, MoveAlongJointPath)
        print 'nodename started'

    def move(self, path, movegroup):
        joint_limits = []

        p = SearchIkSolutionResponse()

        #debug
        print "joint names:"
        print path.joint_trajectory.joint_names
        print "points:"
        print path.joint_trajectory.points[0].positions

        for i in range(len(path.joint_trajectory.joint_names)):
            limit = Limits();
            joint_name = path.joint_trajectory.joint_names[i]
            # if movegroup == "arm":
            cartesian_limits = CartesianLimits()
            cartesian_limits.translational.max_velocity = 0.165
            cartesian_limits.translational.max_acceleration = 4
            cartesian_limits.rotational.max_velocity = 20 * pi / 180.0
            cartesian_limits.rotational.max_acceleration = 100 * pi / 180.0
            limit.max_velocity = 20 * pi / 180.0
            limit.max_acceleration = 400 * pi / 180.0
            # if joint_name[0:3] == "cam":
            #     limit.max_velocity = 0.4
            #     limit.max_acceleration = 2.0
            # if joint_name == "gripper":
            #     limit.max_velocity = gripper_vel
            #     limit.max_acceleration = gripper_acc
            joint_limits.append(limit)

        config = []
        for i in range(len(path.joint_trajectory.points)):
            blub = SearchIkSolutionResponse()
            blub.solution.q = path.joint_trajectory.points[i].positions
            config.append(blub.solution)

        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)

        resp = self.__service(path.joint_trajectory.joint_names, config, ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.stop_reason