#!/usr/bin/env python
from copy import copy, deepcopy

import numpy
import sys
import copy
from math import sqrt
from datetime import time
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Quaternion import Quaternion
from moveit_msgs.msg._CollisionObject import CollisionObject
from numpy.core.multiarray import dot
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
from suturo_perception_msgs.msg._EurocObject import EurocObject
import tf
from tf.transformations import quaternion_from_matrix, rotation_matrix, euler_from_quaternion, quaternion_matrix
import suturo_planning_manipulation.calc_grasp_position
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_box, calculate_grasp_position
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position_cylinder
from suturo_planning_manipulation.calc_grasp_position import get_pre_grasp
from suturo_planning_manipulation.planningsceneinterface import *
from suturo_planning_manipulation.planningsceneinterface import PlanningSceneInterface
from suturo_planning_perception.perception import *
# from suturo_planning_perception.src.suturo_planning_perception.perception import get_gripper_perception
from suturo_planning_plans.visualization import visualize_poses


def test_task1(mani):

    mani.grasp("red_cube")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(0.5, 0.5, 0.00)
    mani.place(dest)

    mani.grasp("green_cylinder")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(0.5, 0, 0)
    mani.place(dest)

    mani.grasp("blue_handle")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(0.5, -0.5, 0)
    mani.place(dest)
    pass

def test_task1_v2(mani):

    mani.grasp("red_cube")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(-0.5, -0.5, 0.00)
    mani.place(dest)

    mani.grasp("cyan_cylinder")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(-0.5, 0, 0)
    mani.place(dest)

    mani.grasp("yellow_handle")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(0.5, -0.5, 0)
    mani.place(dest)

def test_task3(mani):
    mani.grasp_and_move("blue_handle")

    dest = PointStamped()
    dest.header.frame_id = "/odom_combined"
    dest.point = Point(0.85, -0.85, 0)
    mani.place_and_move(dest)

    # mani.grasp_and_move("red_cube")
    #
    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point( -0.85, 0.85, 0.00)
    # mani.place_and_move(dest)

    # mani.grasp_and_move("green_cylinder")
    #
    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point(-0.85, -0.85, 0)
    # mani.place_and_move(dest)

if __name__ == '__main__':
    rospy.init_node('head_mover', anonymous=True)
    # x_axis_unit = Point(1, 0, 0)
    #
    g = PoseStamped()
    g.header.frame_id = "/odom_combined"
    # g.pose.orientation = Quaternion(-0.651200263129, -0.275437965779, 0.651368941659, -0.275301010077)
    g.pose.orientation = euler_to_quaternion(1.1, pi/3, 1.3)
    # print g.pose.orientation
    g.pose.position = Point(0, 0, 0)
    visualize_poses([g])
    # print quaternion_matrix(quaternion_from_euler(0, pi, 0))
    # print dot([[1, 0, 0,0]],quaternion_matrix(quaternion_from_euler(pi/2, pi/2, 0)))
    # a = [[1, 0, 1]]
    # b = [[4], [2], [2]]
    # print dot(b,a)
    # print euler_from_quaternion(quaternion_from_euler(0, pi, 0))
    # print qv_mult(g.pose.orientation, x_axis_unit)
    print get_pitch(g)



    # print (lambda x: a(2, x))(3)
    #
    # #
    #
    # mani = Manipulation()
    #
    # # visualize_poses(move_to_object_cam_pose_in_cool(Point(1, -1, 0), 0.3, pi/2))
    #
    # # redcube task1_v2
    # # t_point = geometry_msgs.msg.PoseStamped()
    # # t_point.header.frame_id = "/odom_combined"
    # # t_point.pose.position = geometry_msgs.msg.Point(0.3, -0.3, 0.015)
    # # t_point.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    # # dist = 0.1
    # # angle = pi/4
    # # mani.object_cam_pose(t_point, dist, angle)
    #
    # #pose = PoseStamped()
    # #pose.header.frame_id = "/odom_combined"
    # #pose.pose.position = Point(0.5, 0, 0.5)
    # # q = quaternion_from_euler(0, pi /4, pi /4)
    # # pose.pose.orientation = Quaternion(*q)
    # # mani.move_to(pose)
    # mani.open_gripper()
    #
    #
    # test_task1(mani)
    # test_task1_v2(mani)


    # print mani.get_arm_move_group().get_current_pose()
    # mani.grasp("red_cube")
    # p = PoseStamped()
    # p.header.frame_id = "/odom_combined"
    # p.pose.position = Point(0, 0, 0)
    # p.pose.position = Point(0.5 , -0.8, 0)
    # mani.move_base(p)


    # mani.move_to("scan_pose3")
    # # mani.turn_arm(0.5*pi)
    # mani.get_planning_scene().remove_object("blue_handle")
    # rospy.sleep(1)
    # a = get_gripper_perception(pose_estimation=True)
    # # print a
    # mani.get_planning_scene().add_object(a[0].mpe_object)

    # pose = PoseStamped()
    # pose.header.frame_id = "/odom_combined"
    # pose.pose.position = Point(0.5, 0, 0.2)
    # pose.pose.orientation = Quaternion(0, 0, 0, 1)
    # obj = mani.get_planning_scene().make_box("muh", pose, [0.05, 0.2, 0.1])
    # mani.get_planning_scene().add_object(obj)
    # mani.grasp("blue_handle")
    # mani.open_gripper()

    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point(0.5, 0.5, 0.00)
    # mani.place(dest)

    # # mani.open_gripper()
    #
    # pose = PoseStamped()
    # pose.header.frame_id = "/tcp"
    # pose.pose.position = Point(0, 0, -post_place_length)
    #
    # mani.move_to(pose)
    # mani.open_gripper()
    # i = 3
    # rospy.loginfo("asd" +str(i))



    # dest = PointStamped()
    # dest.header.frame_id = "/odom_combined"
    # dest.point = Point(-0.85, -0.85, 0)
    # mani.place_and_move(dest)
    # mani.open_gripper()
    # test_task3(mani)
    # test_task1(mani)
    # mani.open_gripper()

    # t_point = geometry_msgs.msg.PoseStamped()
    # t_point.header.frame_id = "/odom_combined"
    # t_point.pose.position = geometry_msgs.msg.Point(-0.3, -0.4, 0.03)
    # t_point.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    #
    # dist = 1
    #
    # angle = pi/4
    #
    # pose = mani.object_cam_pose(t_point, dist, angle)
    #
    # visualize_pose([pose])
    #
    # mani.move_to(pose)
    #
    # print pose

    # co = mani.get_planning_scene().get_collision_object("part4")
    # visualize_point(mani.get_center_of_mass(co))

    # co = mani.get_planning_scene().get_collision_object("red_cube")
    # print mani.calc_object_weight(co, 2710)
    # print mani.get_arm_move_group().get_current_pose()
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