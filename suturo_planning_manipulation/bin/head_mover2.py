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
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_box, calculate_grasp_position
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_cylinder
from suturo_planning_manipulation.calc_grasp_position import get_pre_grasp
from suturo_planning_manipulation.planningsceneinterface import *
from suturo_planning_manipulation.planningsceneinterface import PlanningSceneInterface


# def test_task1(mani):
#     mani.grasp("red_cube")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     # dest.point = Point(-0.3, -0.4, 0.03)
#     dest.point = Point(0.5, 0.5, 0.00)
#     mani.place(dest)
#
#     mani.grasp("green_cylinder")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     dest.point = Point(0.5, 0, 0)
#     mani.place(dest)
#
#     mani.grasp("blue_handle")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     dest.point = Point(0.5, -0.5, 0)
#     mani.place(dest)
#
# def test_task3(mani):
#     pose = PoseStamped()
#     pose.header.frame_id = "/odom_combined"
#     pose.pose.position = Point(0.1858, -0.772, 0.04)
#     pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, -0.1))
#     mani.get_planning_scene().add_box("red_cube", pose, [0.05, 0.05, 0.05])
#
#     mani.grasp_and_move("blue_handle")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     dest.point = Point(0.85, -0.85, 0)
#     mani.place_and_move(dest)
#
#     mani.grasp_and_move("red_cube")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     # dest.point = Point(-0.3, -0.4, 0.03)
#     dest.point = Point( -0.85, 0.85, 0.00)
#     mani.place_and_move(dest)
#
#     mani.grasp_and_move("green_cylinder")
#
#     dest = PointStamped()
#     dest.header.frame_id = "/odom_combined"
#     dest.point = Point(-0.85, -0.85, 0)
#     mani.place_and_move(dest)



if __name__ == '__main__':
    rospy.init_node('head_mover', anonymous=True)

    mani = Manipulation()
    # test_task3(mani)

    num_of_scans = 12
    max_rad = 5.9
    rad_per_step = max_rad / num_of_scans

    #x is zwischen 0 und num_of_scans
    x = 5
    rad = x * rad_per_step - 2.945
    rospy.loginfo('Turning arm %s' % str(rad))
    mani.turn_arm(rad)

    # co = mani.get_planning_scene().get_collision_object("red_cube")
    # print mani.calc_object_weight(co, 2710)
    # print mani.get_arm_move_group().get_current_joint_values()
    #
    # mani = Manipulation()
    # ogog = geometry_msgs.msg.PoseStamped()
    # ogog.header.frame_id = "/odom_combined"
    # ogog.header.stamp = rospy.Time.now()
    # ogog.pose.position = geometry_msgs.msg.Point(0.0, 0.1, 0.05)
    # ogog.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
    # mani.move_base(ogog)
    # mani.grasp("red_cube")
    # test_task1(mani)
    # mani.open_gripper()
    # mani.grasp("blue_handle")
    # p = PoseStamped()
    # p2 = PoseStamped()
    # p.header.stamp = rospy.Time.now()
    # p.header.frame_id = "/odom_combined"
    # p.pose.position = Point(0, 0, 0.9)
    # q = quaternion_from_euler(0, 0, pi)
    # p.pose.orientation = Quaternion(*q)
    #
    # # angle = quaternion_from_euler(0, pi / 2, 0)
    # # o = Quaternion(*q)
    # # no = quaternion_multiply([o.x, o.y, o.z, o.w], angle)
    # #
    # #
    # # p2 = deepcopy(p)
    # #
    # # p2.pose.orientation = geometry_msgs.msg.Quaternion(*no)
    #
    # # p.pose.orientation = Quaternion(0, 0, 0, 1)
    # # mani.set_constraint(p2)
    # print "moving"
    # # mani.move_to(p, False)
    # mani.move_to(p, True)

    # mani.turn_arm(0, 0, 6)
    # print "muh"
    # mani.open_gripper()
    # mani.move_to("scan_pose1")
    # mani.turn_arm(0, -2.9, 6)
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