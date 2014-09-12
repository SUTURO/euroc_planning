#!/usr/bin/env python
from copy import copy, deepcopy

import numpy
import sys
import copy
from math import sqrt
from geometry_msgs.msg._PointStamped import PointStamped
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import tf
from tf.transformations import quaternion_from_matrix, rotation_matrix
import suturo_planning_manipulation.calc_grasp_position
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_box, visualize_pose, calculate_grasp_position
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_cylinder
from suturo_planning_manipulation.calc_grasp_position import get_pre_grasp
from suturo_planning_manipulation.planningsceneinterface import *
from suturo_planning_manipulation.planningsceneinterface import PlanningSceneInterface


if __name__ == '__main__':
    rospy.init_node('head_mover', anonymous=True)
    # co = moveit_msgs.msg.CollisionObject()
    # co.id = "green_cylinder"
    # co.header.frame_id = "/odom_combined"
    # co.primitives = []
    # co.primitives.append(shape_msgs.msg.SolidPrimitive())
    # co.primitives[0].type = shape_msgs.msg.SolidPrimitive.CYLINDER
    # co.primitives[0].dimensions.append(0.1)
    # co.primitives[0].dimensions.append(0.1)
    # co.primitives[0].dimensions.append(0.05)
    # co.primitive_poses = []
    # co.primitive_poses.append(geometry_msgs.msg.Pose())
    # co.primitive_poses[0].position = geometry_msgs.msg.Point(1, 1, 0)
    # co.primitive_poses[0].orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)

    # a = moveit_commander.PlanningSceneInterface()
    # rospy.sleep(2)
    # muh = geometry_msgs.msg.PoseStamped()
    # muh.header.frame_id = "/odom_combined"
    # muh.pose.position = geometry_msgs.msg.Point(0, 0, 0)
    # muh.pose.orientation = orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    # a.add_plane("muh", muh)
    # rospy.sleep(2)
    # ps = PlanningSceneInterface()
    # print ps.get_collision_objects()

    mani = Manipulation()
    ogog = geometry_msgs.msg.PoseStamped()
    ogog.header.frame_id = "/odom_combined"
    ogog.header.stamp = rospy.Time.now()
    ogog.pose.position = geometry_msgs.msg.Point(0.0, 0.1, 0.05)
    ogog.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
    mani.move_base(ogog)
    # mani.grasp("red_cube")

    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # # dest.point = Point(-0.3, -0.4, 0.03)
    # dest.point = Point(0.5, 0.5, 0.00)
    # mani.place(dest)

    # mani.grasp("green_cylinder")

    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point(0.5, 0, 0)
    # mani.place(dest)

    # mani.grasp("blue_handle")

    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point(0.5, -0.5, 0)
    # mani.place(dest)

    # mani.turn_arm(0, 0, 6)
    # print "muh"
    # mani.open_gripper()
    # mani.move_to("scan_pose1")
    # mani.turn_arm(0, -2.9)
    # mani.turn_arm(0, pi/2)
    # mani.turn_arm(0, pi)
    # mani.turn_arm(0, pi * 1.5)
    # mani.turn_arm(0, 5.9)
    # while (not rospy.is_shutdown()):
    #     print mani.get_arm_move_gourp().get_current_joint_values()
    #     rospy.sleep(1)

    # mani.stop()
    # rospy.sleep(2)
    # muh = calculate_grasp_position_cylinder(co)
    # for g in muh:
    #     print g.pose.position
    #     print ""
    # print "==============================="
    # mani.sort_grasps(muh)
    # for g in muh:
    #     print g.pose.position
    #     print ""
    # map(get_pre_grasp, muh, asd)
    # visualize_grasps(map(get_pre_grasp, muh))

    # visualize_grasps(calculate_grasp_position_box(co))
    # rospy.sleep(1)

    # visualize_grasps(calculate_grasp_position(co))

    # mani.grasp("red_cube")
    # mani.grasp("green_cylinder")
    # mani.grasp("blue_handle")
    # mani.open_gripper()
    # mani.close_gripper("green_cylinder")
    # print mani.get_ps().get_collision_object("muh1")
    # mani.get_planning_scene().remove_object("red_cube1")
    # ps = moveit_commander.PlanningSceneInterface()
    # muh = geometry_msgs.msg.PoseStamped()
    # muh.header.frame_id= "/odom_combined"
    # muh.pose.position = geometry_msgs.msg.Point(1, 0, 0)
    # muh.pose.orientation = orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    # # ps.remove_world_object("muh")
    # # rospy.sleep(0.5)
    # ps.add_box("muh", muh, (0.1, 0.1, 0.5))
    # print mani.get_ps().get_collision_objects()
    # mani.grasp(co)
    # mani.open_gripper()
    # mani.close_gripper("green_cylinder")
    # rospy.sleep(2)
    # print "muh??"
    # mani.close_gripper()
    # co2 = deepcopy(co)
    # co2.type = 23
    # print co
    # print co2

        # while not rospy.is_shutdown():
    #     print mani.get_arm_move_gourp(  ).get_current_joint_values()
    #     rospy.sleep(0.5)