#!/usr/bin/env python
from math import pi

import sys
import copy
from numpy.core.multiarray import ndarray
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from calc_grasp_position import *
from planningsceneinterface import *

gripper_max_pose = 0.03495


class Manipulation(object):
    def __init__(self):
        self.__listener = tf.TransformListener()
        moveit_commander.roscpp_initialize(sys.argv)
        self.__group = moveit_commander.MoveGroupCommander("arm")
        self.__group.set_planning_time(2)
        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.__planning_scene_interface = PlanningSceneInterface()
        # self.__gripper_max_pose = 0.03495
        rospy.sleep(2)

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move_to(self, goal_pose):
        visualize_grasps([goal_pose])
        angle = quaternion_from_euler(0, pi / 2, 0)
        o = goal_pose.pose.orientation
        no = quaternion_multiply([o.x, o.y, o.z, o.w], angle)
        goal_pose.pose.orientation = geometry_msgs.msg.Quaternion(*no)

        goal_pose = self.transform(goal_pose)
        self.__group.set_pose_target(goal_pose)

        return self.__group.go()

    def transform(self, pose_target):
        try:
            return self.__listener.transformPose("/odom_combined", pose_target)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None

    def open_gripper(self, position=gripper_max_pose):
        self.__gripper_group.set_joint_value_target([-position, position])
        self.__gripper_group.go()
        self.__gripper_group.detach_object()

    def close_gripper(self, object_name=""):
        if object_name != "":
            self.__gripper_group.attach_object(object_name, "gp", ["gp", "finger1", "finger2"])
            rospy.sleep(0.5)
        self.__gripper_group.set_joint_value_target([0.0, 0.0])
        self.__gripper_group.go()

    def grasp(self, collision_object):
        if type(collision_object) is str:
            collision_object = self.__planning_scene_interface.get_collision_object(collision_object)
        grasp_positions = calculate_grasp_position(collision_object)
        self.sort_grasps(grasp_positions)
        print len(grasp_positions)
        self.open_gripper()
        for grasp in grasp_positions:
            if self.move_to(get_pre_grasp(grasp)):

                if not self.move_to(grasp):
                    continue

                self.close_gripper(collision_object.id)
                print "grasped"
                break

    def place(self, destination):
        
        pass


    def sort_grasps(self, grasps):
        grasps.sort(key=self.__grasp_value)

    def __grasp_value(self, grasp):
        return -self.transform(grasp).pose.position.z

    def get_planning_scene(self):
        return self.__planning_scene_interface