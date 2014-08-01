#!/usr/bin/env python
from array import array
from copy import deepcopy
from math import sqrt, pi, cos, sin, acos
import numpy

import sys
import copy
import geometry_msgs.msg
from geometry_msgs.msg._Point import Point
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import tf
from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_from_euler, quaternion_multiply
import visualization_msgs.msg

finger_length = 0.06
hand_length = 0.1535
pre_grasp_length = 0.09


def calculate_grasp_position(collision_object):
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().BOX:
        return calculate_grasp_position_box(collision_object)
    if collision_object.primitives[0].type == shape_msgs.msg.SolidPrimitive().CYLINDER:
        return calculate_grasp_position_cylinder(collision_object)
    print "cant compute grasp position for this type of shit"


def get_pre_grasp(grasp):
    pre_grasp = deepcopy(grasp)
    depth = magnitude(pre_grasp.pose.position)
    normalize(pre_grasp.pose.position)
    depth += pre_grasp_length
    pre_grasp.pose.position = multiply_point(depth, pre_grasp.pose.position)
    return pre_grasp


def calculate_grasp_position_box(collision_object):
    points = make_points_around_box()
    grasp_positions = []

    depth = finger_length
    if finger_length < collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X]:
        depth = collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X]
    depth += hand_length

    grasp_positions.append(make_grasp_pose(depth, points[0], points[1], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[0], points[2], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, points[1], points[0], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[1], points[2], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, points[2], points[0], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[2], points[1], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, points[3], points[4], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[3], points[5], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, points[4], points[3], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[4], points[5], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, points[5], points[3], collision_object.id))
    grasp_positions.append(make_grasp_pose(depth, points[5], points[4], collision_object.id))

    return grasp_positions


def make_grasp_pose(depth, p1, p2, frame_id):
    grasp = geometry_msgs.msg.PoseStamped()
    grasp.header.frame_id = frame_id
    grasp.pose.orientation = three_points_to_quaternion(p1, geometry_msgs.msg.Point(0, 0, 0), p2)
    grasp.pose.position = multiply_point(depth, p1)
    return grasp


def make_points_around_box():
    points = []
    points.append(geometry_msgs.msg.Point(1, 0, 0))
    points.append(geometry_msgs.msg.Point(0, 1, 0))
    points.append(geometry_msgs.msg.Point(0, 0, 1))

    points.append(geometry_msgs.msg.Point(-1, 0, 0))
    points.append(geometry_msgs.msg.Point(0, -1, 0))
    points.append(geometry_msgs.msg.Point(0, 0, -1))

    return points


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

    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, 1), Point(1, 0, 0), collision_object.id))
    # grasp_positions.append(make_grasp_pose(depth, points[2], points[1], collision_object.id))

    grasp_positions.append(make_grasp_pose(depth, Point(0, 0, -1), Point(1, 0, 0), collision_object.id))
    # grasp_positions.append(make_grasp_pose(depth, points[5], points[4], collision_object.id))

    for i in range(0, n):
        a = 2 * pi * ((i+0.0) /(n+0.0))
        # print ((i+0.0) /(n+0.0))
        # print Point(cos(a), sin(a), 0)
        grasp_positions.append(make_grasp_pose(depth_side, Point(cos(a), sin(a), 0), Point(0, 0, 1), collision_object.id))
    grasp_positions.sort()
    return grasp_positions

def get_angle(p1, p2):
    return acos(scalar_product(p1, p2) / (sqrt(scalar_product(p1, p1)) * sqrt(scalar_product(p2, p2))));


def three_points_to_quaternion(origin, to, roll):
    n_1 = subtract_point(to, origin)
    normalize(n_1)

    n = subtract_point(roll, origin)
    normalize(n)

    n_2 = subtract_point(n, multiply_point(scalar_product(n, n_1), n_1))
    normalize(n_2)

    n_3 = cross_product(n_1, n_2)
    normalize(n_3)

    rm = numpy.array(((n_1.x, n_2.x, n_3.x, 0),
                      (n_1.y, n_2.y, n_3.y, 0),
                      (n_1.z, n_2.z, n_3.z, 0),
                      (0, 0, 0, 1)), dtype=numpy.float64)

    q = quaternion_from_matrix(rm)

    return geometry_msgs.msg.Quaternion(*q)


def subtract_point(p1, p2):
    if type(p1) is geometry_msgs.msg.Point and type(p2) is geometry_msgs.msg.Point:
        result = geometry_msgs.msg.Point()
        result.x = p1.x - p2.x
        result.y = p1.y - p2.y
        result.z = p1.z - p2.z
        return result


def normalize(p):
    if p.x == 0 and p.y == 0 and p.z == 0:
        return None
    a = 1 / sqrt((p.x * p.x + p.y * p.y + p.z * p.z))
    p.x *= a
    p.y *= a
    p.z *= a


def multiply_point(s, p):
    result = geometry_msgs.msg.Point()
    result.x = p.x * s
    result.y = p.y * s
    result.z = p.z * s
    return result


def scalar_product(p1, p2):
    return p1.x * p2.x + p1.y * p2.y + p1.z * p2.z


def magnitude(q):
    if type(q) is geometry_msgs.msg.Quaternion:
        return sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    else:
        return sqrt(q.x * q.x + q.y * q.y + q.z * q.z)


def cross_product(p1, p2):
    result = geometry_msgs.msg.Point()
    result.x = p1.y * p2.z - p1.z * p2.y
    result.y = p1.z * p2.x - p1.x * p2.z
    result.z = p1.x * p2.y - p1.y * p2.x
    return result


def visualize_grasps(grasps):
    pub = rospy.Publisher('visualization_marker', visualization_msgs.msg.Marker, queue_size=10)
    rospy.sleep(1)
    r = rospy.Rate(1)  # 10hz

    marker = visualization_msgs.msg.Marker()
    marker.header.frame_id = grasps[0].header.frame_id
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "mani"
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.scale.z = 0.01
    marker.scale.y = 0.02
    marker.scale.x = 0.03

    i = 0
    for grasp in grasps:
        marker.id = i
        marker.pose = grasp.pose
        i += 1
        # rospy.loginfo(marker)
        pub.publish(marker)
        r.sleep()


if __name__ == '__main__':
    print "muh"


























