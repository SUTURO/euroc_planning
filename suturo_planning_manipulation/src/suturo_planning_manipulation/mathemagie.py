from numpy.core.multiarray import dot
__author__ = 'ichumuh'

from array import array
from copy import deepcopy
from math import sqrt, pi, cos, sin, acos
import numpy

import sys
import copy
import geometry_msgs.msg
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import shape_msgs.msg
import tf
from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_from_euler, quaternion_multiply, \
    quaternion_conjugate, quaternion_matrix
import visualization_msgs.msg
from manipulation_constants import *
from geometry_msgs.msg._PoseStamped import PoseStamped

def get_angle(p1, p2):
    return acos(scalar_product(p1, p2) / (sqrt(scalar_product(p1, p1)) * sqrt(scalar_product(p2, p2))))


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
    # assert (type(p1) is geometry_msgs.msg.Point, "p1 is not of type Point")
    # assert (type(p2) is geometry_msgs.msg.Point, "p2 is not of type Point")
    result = geometry_msgs.msg.Point()
    result.x = p1.x - p2.x
    result.y = p1.y - p2.y
    result.z = p1.z - p2.z
    return result


def add_point(p1, p2):
    # assert (type(p1) is geometry_msgs.msg.Point, "p1 is not of type Point")
    # assert (type(p2) is geometry_msgs.msg.Point, "p2 is not of type Point")
    result = geometry_msgs.msg.Point()
    result.x = p1.x + p2.x
    result.y = p1.y + p2.y
    result.z = p1.z + p2.z
    return result


def normalize(p):
    if p.x == 0 and p.y == 0 and p.z == 0:
        return None
    a = 1 / sqrt((p.x * p.x + p.y * p.y + p.z * p.z))
    p.x *= a
    p.y *= a
    p.z *= a

def set_vector_length(l, p):
    p2 = deepcopy(p)
    normalize(p2)
    return multiply_point(l, p2)

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
        return sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    else:
        return sqrt(q.x**2 + q.y**2 + q.z**2)


def cross_product(p1, p2):
    result = geometry_msgs.msg.Point()
    result.x = p1.y * p2.z - p1.z * p2.y
    result.y = p1.z * p2.x - p1.x * p2.z
    result.z = p1.x * p2.y - p1.y * p2.x
    return result

def rotate_quaternion(q, r, p, y):
    angle = quaternion_from_euler(r, p, y)
    no = quaternion_multiply([q.x, q.y, q.z, q.w], angle)
    return Quaternion(*no)

def euler_to_quaternion(r, p, y):
    return Quaternion(*quaternion_from_euler(r, p, y))

def get_pitch(q):
    gripper_direction = qv_mult(q, Point(1, 0, 0))
    # print gripper_direction
    v = deepcopy(gripper_direction)

    # visualize_point(v)
    # print v
    # print magnitude(gripper_direction)
    v.z = 0
    # b =  Point(1, 0, 0)
    # print b
    # gripper_direction.y = 0
    if 0 <= magnitude(v) <= 0.001:
        v = Point(1, 0, 0)
    pitch = get_angle(v, gripper_direction)
    return pitch

    # v = deepcopy(gripper_direction)
    # v.x = sqrt(v.x**2 + v.y**2) * 1 if v.x > 0 else -1
    # v.y = 0
    #
    # if 0 <= magnitude(v) <= 0.001:
    #     v = Point(1, 0, 0)
    # v2 = Point(1, 0, 0)
    # pitch = get_angle(v, v2)
    # return pitch

# def get_yaw(dest):
#     x = Point(1, 0, 0)
#
#     d = dest
#     d.z = 0
#
#     alpha = get_angle(x, d)
#     if d.y < 0:
#         alpha = -alpha
#     return alpha


def normalize2(v, tolerance=0.00001):
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def qv_mult(q1, v1):
    q = q1
    v = v1
    if type(q1) is Quaternion:
        q = (q1.x, q1.y, q1.z, q1.w)
    if type(v1) is Point:
        v = (v1.x, v1.y, v1.z, 0)

    r = quaternion_multiply(quaternion_multiply(q, v), quaternion_conjugate(q))
    return Point(r[0], r[1], r[2])


def orientation_to_vector(orientation):
    return qv_mult(orientation, Point(1, 0, 0))

