#!/usr/bin/env python
from suturo_planning_manipulation.manipulation_constants import gripper_max_pose

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
        self.__move_service = rospy.ServiceProxy(servicename, MoveAlongJointPath)
        rospy.wait_for_service('/euroc_interface_node/get_timing_along_joint_path')
        self.__timing_service = rospy.ServiceProxy(servicename, GetTimingAlongJointPath)

        self.tcp_limits = CartesianLimits()
        self.tcp_limits.translational.max_velocity = 0.18
        self.tcp_limits.translational.max_acceleration = 4
        self.tcp_limits.rotational.max_velocity = 10 * pi / 180.0
        self.tcp_limits.rotational.max_acceleration = 100 * pi / 180.0

        self.lwr_max_velocity = 0.37
        self.lwr_max_acceleration = 5.3
        self.axis_max_velocity = 0.165
        self.axis_max_acceleration = 4
        self.finger_max_velocity = 0.05
        self.finger_max_acceleration = 15

    def set_turbo_mode(self, factor=1.75):
        self.tcp_limits.translational.max_velocity *= factor
        self.tcp_limits.translational.max_acceleration *= factor
        self.tcp_limits.rotational.max_velocity *= factor
        self.tcp_limits.rotational.max_acceleration *= factor
        self.lwr_max_velocity *= factor
        self.lwr_max_acceleration *= factor
        self.axis_max_velocity *= factor
        self.axis_max_acceleration *= factor
        self.finger_max_velocity = 0.5
        self.finger_max_acceleration *= factor


    def set_joint_limits(self, joint_names):
        joint_limits = []
        for i in range(len(joint_names)):
            limit = Limits()
            joint_name = joint_names[i]
            if joint_name.startswith("lwr"):
                limit.max_velocity = self.lwr_max_velocity #20 * pi / 180.0
                limit.max_acceleration = self.lwr_max_acceleration #300 * pi / 180.0
            if joint_name.startswith("axis"):
                limit.max_velocity = self.axis_max_velocity
                limit.max_acceleration = self.axis_max_acceleration
            if joint_name.startswith("joint_before_finger1"):
                limit.max_velocity = self.finger_max_velocity
                limit.max_acceleration = self.finger_max_acceleration
            joint_limits.append(limit)
        return joint_limits

    def move(self, path):
        # check if moveit generated a trajectory
        if len(path.joint_trajectory.points) == 0:
            return False

        # for every joint in the list, create the appropriate limits and store them
        joint_limits = self.set_joint_limits(path.joint_trajectory.joint_names)

        # create the configuration path for the joints
        config = []
        # format the configuration for the gripper
        if path.joint_trajectory.joint_names[0].startswith("joint_before_finger"):
            # rename jointname
            path.joint_trajectory.joint_names = ["gripper"]
            # remove the second entry from the list as the service expects only one joint
            joint_limits.pop()
            # copy every configuration from the trajectory
            for i in range(len(path.joint_trajectory.points)):
                blub = Configuration()
                # because the gripper is only one joint but moveit returns two, we take the second value and double it
                blub.q = [2 * path.joint_trajectory.points[i].positions[1]]
                if blub.q[0] > gripper_max_pose*2:
                    blub.q[0] = gripper_max_pose*2
                config.append(blub)
        else:
            # copy the configuration generated by moveit into the expected object
            for i in range(len(path.joint_trajectory.points)):
                if i % 2 == 0:
                    blub = Configuration()
                    blub.q = path.joint_trajectory.points[i].positions
                    config.append(blub)

        # set the time
        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)

        # call the service and store the response
        # rospy.logdebug("calling self.__move_service with "+str(path.joint_trajectory.joint_names) +", "+ str(config)+", "+str(ros_start_time)+", "+str(joint_limits)+", "+str(self.tcp_limits))
        resp = self.__move_service(path.joint_trajectory.joint_names, config, ros_start_time, joint_limits, self.tcp_limits)
        # rospy.logdebug("manipulation service move move_service return value = "+str(resp))
        if resp.error_message:
            raise ManipulationServiceException(resp.error_message)
        if resp.stop_reason == "path finished":
            return True
        raise ManipulationServiceException(resp.error_message)

    def direct_move(self, configuration):

        # joint_limits = []
        joint_names = []
        # for every joint in the list, create the appropriate limits and store them
        for i in range(1, 8):
            joint_name = "lwr_joint_" + i.__str__()
            joint_names.append(joint_name)

        joint_limits = self.set_joint_limits(joint_names)
            # limit = Limits()
            # limit.max_velocity = 20 * pi / 180.0
            # limit.max_acceleration = 400 * pi / 180.0
            # joint_limits.append(limit)

        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)

        # call the service and store the response
        resp = self.__move_service(joint_names, [configuration], ros_start_time, joint_limits, self.tcp_limits)
        if resp.error_message:
            raise ManipulationServiceException(resp.error_message)
        if resp.stop_reason == "path finished":
            return True
        raise ManipulationServiceException(resp.error_message)

    def get_timing(self, path):
        # check if moveit generated a trajectory
        if len(path.joint_trajectory.points) == 0:
            return False

        joint_limits = self.set_joint_limits(path.joint_trajectory.joint_names)

        # create the configuartionpath for the joints
        config = []
        # format the configuration for the gripper
        if path.joint_trajectory.joint_names[0].startswith("joint_before_finger"):
            # rename jointname
            path.joint_trajectory.joint_names = ["gripper"]
            # remove the second entry from the list as the service expects only one joint
            joint_limits.pop()
            # copy every configuration from the trajectory
            for i in range(len(path.joint_trajectory.points)):
                blub = SearchIkSolutionResponse()
                # because the gripper is only one joint but moveit returns two, we take the second value and double it
                blub.solution.q = [2 * path.joint_trajectory.points[i].positions[1]]
                if blub.solution.q[0] > 0.065:
                    blub.solution.q[0] = 0.065
                config.append(blub.solution)
        else:
            # copy the configuration generated by moveit into the expected object
            for i in range(len(path.joint_trajectory.points)):
                blub = SearchIkSolutionResponse()
                blub.solution.q = path.joint_trajectory.points[i].positions
                config.append(blub.solution)

        # set the time
        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)

        # call the service and store the response
        resp = self.__timing_service(path.joint_trajectory.joint_names, config, joint_limits, self.tcp_limits)
        return resp.time_at_via_point

    def pan_tilt(self, pan, tilt):
        cartesian_limits = CartesianLimits()
        cartesian_limits.translational.max_velocity = 0
        cartesian_limits.translational.max_acceleration = 0
        cartesian_limits.rotational.max_velocity = 0
        cartesian_limits.rotational.max_acceleration = 0
        joint_limits = []
        for i in range(0, 2):
            limit = Limits()
            limit.max_velocity = 0.4
            limit.max_acceleration = 2.0
            joint_limits.append(limit)
        config = []
        p = SearchIkSolutionResponse()
        p.solution.q = [pan, tilt]
        config.append(p.solution)
        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(0)
        resp = self.__move_service(['cam_pan', 'cam_tilt'], config, ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.stop_reason


# class for our own exception
class ManipulationServiceException(Exception):
    pass