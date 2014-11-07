#!/usr/bin/env python
from array import array
from copy import deepcopy
from math import sqrt, pi, cos, sin, acos
import numpy

import sys
import copy
import geometry_msgs.msg
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PointStamped import PointStamped
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
from suturo_planning_visualization.visualization import visualize_point


def calculate_grasp_position(collision_object, transform_func, n=8):
    '''
    Calculates grasp positions for the given collision object
    :param collision_object: CollisionObject
    :param transform_func(object, frame_id): a function to transform objects to different frame_ids. (use Manipulation.transform_to)
    :param n: number of grasps around the side if it is a cylinder or every side ob the box
    :return: list of PoseStamped
    '''
    if len(collision_object.primitives) > 1:
        return calculate_grasp_position_list(collision_object, transform_func)
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().BOX:
        return calculate_grasp_position_box(collision_object, n)
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().CYLINDER:
        return calculate_grasp_position_cylinder(collision_object, n)
    print "Can't compute grasp position for this type."


def calculate_grasp_position_list(collision_object, transform_func):
    '''
    Calculates grasp positions for a composition of collision objects
    :param collision_object: CollisionObject
    :param transform_func(object, frame_id): transform function
    :return: list of PoseStamped
    '''
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
    '''
    Returns a position that should be taken before grasping.
    :param grasp: PoseStamped
    :return: PoseStamped that is further behind
    '''
    pre_grasp = deepcopy(grasp)
    p = get_fingertip(grasp).point
    grasp_dir = subtract_point(grasp.pose.position, p)
    grasp_dir = set_vector_length(pre_grasp_length, grasp_dir)
    pre_grasp.pose.position = add_point(grasp.pose.position, grasp_dir)
    return pre_grasp


def calculate_grasp_position_box(collision_object, n=8):
    '''
    Calculates grasp positions for a Box.
    :param collision_object: CollisionObject
    :param n: number of grasps around each side
    :return: list of PoseStamped
    '''
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
    #TODO: cooler machen
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


def make_grasp_pose(depth, gripper_origin, roll, frame_id):
    '''
    Calculates a Pose pointing from gripper_origin to direction.
    :param depth: desired distance between gripper_origin and direction
    :param gripper_origin: PoseStamped
    :param roll: PoseStamped
    :param frame_id: str
    :return: PoseStamped
    '''
    grasp = PoseStamped()
    grasp.header.frame_id = frame_id
    grasp.pose.orientation = three_points_to_quaternion(gripper_origin, geometry_msgs.msg.Point(0, 0, 0), roll)
    grasp.pose.position = multiply_point(depth, gripper_origin)
    return grasp

def calculate_grasp_position_cylinder(collision_object, n=4):
    '''
    Calculates grasp positions for a Cylinder.
    :param collision_object: CollisionObject
    :param n: number of grasp around the side
    :return: list of PoseStamped
    '''
    grasp_positions = []

    d1 = finger_length
    h = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT]
    if finger_length < h:
        d1 = h
    # depth += hand_length

    d2 = finger_length
    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]:
        d2 = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]
    # depth_side += hand_length

    #TODO: abfangen wenn eine Seite zu gross ist
    #Points above and below the cylinder
    depth = d1 + hand_length
    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, 1), Point(0, 1, 0), collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, 1), Point(1, 0, 0), collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, -1), Point(0, 1, 0), collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, -1), Point(1, 0, 0), collision_object.id))

    #Points around the side of the cylinder
    depth = d2 + hand_length
    grasp_positions.extend(make_scan_pose(Point(0,0,0), depth, 0, collision_object.id, 4))
    grasp_positions.extend(make_scan_pose(Point(0,0,h/2-0.03), depth, 0, collision_object.id, 4))
    grasp_positions.extend(make_scan_pose(Point(0,0,-(h/2-0.03)), depth, 0, collision_object.id, 4))

    # d3 = sqrt(d1**2 + d2**2)
    # depth = d1 + hand_length
    grasp_positions.extend(make_scan_pose(Point(0,0,h/2-0.03), depth, pi/4, collision_object.id, 8))
    grasp_positions.extend(make_scan_pose(Point(0,0,-(h/2-0.03)), depth, -pi/4, collision_object.id, 8))

    return grasp_positions


def make_scan_pose(point, distance, angle, frame="/odom_combined", n=8):
    '''
    Calculates "n" positions around and pointing to "point" with "distance" in an "angle"
    :param point: Point the positions will be pointing to
    :param distance: float
    :param angle: float, 0 = horizontally, pi/2 = downwards
    :param frame: str, the frame that the positions will have
    :param n: float, number of positions
    :return: list of PoseStamped
    '''
    #TODO: assumes odom_combined
    look_positions = []

    alpha = pi/2 - angle
    r = sin(alpha) * distance
    h = cos(alpha) * distance
    # print "h ", h
    h_vector = Point()
    h_vector.z = h
    muh = add_point(point, h_vector)
    for i in range(0, n):
        a = 2 * pi * ((i + 0.0) / (n + 0.0))
        b = a - (pi / 2)

        look_point = Point(cos(a), sin(a), 0)
        look_point = set_vector_length(r, look_point)
        look_point = add_point(look_point, muh)

        roll_point = Point(cos(b), sin(b), 0)
        roll_point = add_point(roll_point, point)
        look_pose = PoseStamped()
        look_pose.header.frame_id = frame
        look_pose.pose.orientation = three_points_to_quaternion(look_point, point, roll_point)

        look_pose.pose.position = look_point
        look_positions.append(look_pose)

    look_positions.sort(key=lambda x: magnitude(x.pose.position))
    return look_positions

























