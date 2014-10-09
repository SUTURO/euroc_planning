#!/usr/bin/env python
from array import array
from copy import deepcopy
from math import sqrt, pi, cos, sin, acos
import numpy

import sys
import copy
import geometry_msgs.msg
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import tf
from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_from_euler, quaternion_multiply
import visualization_msgs.msg
from manipulation_constants import *
from mathemagie import *


def calculate_grasp_position(collision_object, transform_func, n=8):
    if len(collision_object.primitives) > 1:
        return calculate_grasp_position_list(collision_object, transform_func)
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().BOX:
        return calculate_grasp_position_box(collision_object, n)
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().CYLINDER:
        return calculate_grasp_position_cylinder(collision_object, n)
    print "Can't compute grasp position for this type."


def calculate_grasp_position_list(collision_object, transform_func):
    grasp_positions = []
    for i in range(0, len(collision_object.primitives)):
        co = CollisionObject()
        co.id = collision_object.id
        co.primitives.append(collision_object.primitives[i])
        co.header = collision_object.header
        co.primitive_poses.append(collision_object.primitive_poses[i])
        temp = calculate_grasp_position(co, transform_func, 4)

        for j in range(0, len(temp)):
            tmp_pose = transform_func(co, "/" + collision_object.id).primitive_poses[0].position
            # a = subtract_point(tmp_pose, collision_object.primitive_poses[0].position)
            # if j == 0:
            # print tmp_pose
            temp[j].pose.position = add_point(temp[j].pose.position, tmp_pose)

        grasp_positions.extend(temp)

    # print grasp_positions

    return grasp_positions


def get_pre_grasp(grasp):
    #TODO: nur z beachten
    pre_grasp = deepcopy(grasp)
    depth = magnitude(pre_grasp.pose.position)
    normalize(pre_grasp.pose.position)
    depth += pre_grasp_length
    pre_grasp.pose.position = multiply_point(depth, pre_grasp.pose.position)
    return pre_grasp


def calculate_grasp_position_box(collision_object, n=8):
    grasp_positions = []

    depth = finger_length
    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X]:
        depth = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X]
    depth_x = depth + hand_length

    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y]:
        depth = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y]
    depth_y = depth + hand_length

    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z]:
        depth = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z]
    depth_z = depth + hand_length


    #TODO: abfangen wenn eine Seite zu gross ist
    for i in range(0, n):
        a = 2 * pi * ((i + 0.0) / (n + 0.0))

        grasp_point = Point(cos(a), sin(a), 0)
        d = (abs(grasp_point.x) * depth_x + abs(grasp_point.y) * depth_y + abs(grasp_point.z) * depth_z) / (abs(
            grasp_point.x) + abs(grasp_point.y) + abs(grasp_point.z))
        # print d
        grasp_positions.append(make_grasp_pose(d, grasp_point, Point(0,0,1),
                                               collision_object.id))

        grasp_point = Point(cos(a), 0, sin(a))
        d = (abs(grasp_point.x) * depth_x + abs(grasp_point.y) * depth_y + abs(grasp_point.z) * depth_z) / (abs(
            grasp_point.x) + abs(grasp_point.y) + abs(grasp_point.z))
        grasp_positions.append(make_grasp_pose(d, grasp_point, Point(0,1,0),
                                               collision_object.id))

        grasp_point = Point(0, cos(a), sin(a))
        d = (abs(grasp_point.x) * depth_x + abs(grasp_point.y) * depth_y + abs(grasp_point.z) * depth_z) / (abs(
            grasp_point.x) + abs(grasp_point.y) + abs(grasp_point.z))
        grasp_positions.append(make_grasp_pose(d, grasp_point, Point(1,0,0),
                                               collision_object.id))

    return grasp_positions


def make_grasp_pose(depth, p1, p2, frame_id):
    grasp = PoseStamped()
    grasp.header.frame_id = frame_id
    grasp.pose.orientation = three_points_to_quaternion(p1, geometry_msgs.msg.Point(0, 0, 0), p2)
    grasp.pose.position = multiply_point(depth, p1)
    return grasp

def calculate_grasp_position_cylinder(collision_object, n=8):
    # points = make_points_around_box()
    grasp_positions = []

    depth = finger_length
    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT]:
        depth = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT]
    depth += hand_length

    depth_side = finger_length
    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]:
        depth_side = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]
    depth_side += hand_length
    # print depth

    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, 1), Point(0, 1, 0), collision_object.id))
    # grasp_positions.append(make_grasp_pose(depth, points[2], points[1], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, -1), Point(0, 1, 0), collision_object.id))
    # grasp_positions.append(make_grasp_pose(depth, points[5], points[4], collision_object.id))

    for i in range(0, n):
        a = 2 * pi * ((i + 0.0) / (n + 0.0))
        b = a + (pi / 4)
        # print ((i+0.0) /(n+0.0))
        # print Point(cos(a), sin(a), 0)
        grasp_positions.append(make_grasp_pose(depth_side, Point(cos(a), sin(a), 0), Point(cos(b), sin(b), 0),
                                               collision_object.id))
    grasp_positions.sort()
    return grasp_positions


























