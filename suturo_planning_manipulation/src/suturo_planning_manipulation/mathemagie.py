from numpy.core.multiarray import dot
__author__ = 'ichumuh'

from copy import deepcopy
from math import sqrt, pi, cos, sin, acos
import numpy as np

import geometry_msgs.msg
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion
import geometry_msgs.msg
from tf.transformations import quaternion_from_matrix, rotation_matrix, quaternion_from_euler, quaternion_multiply, \
    quaternion_conjugate, quaternion_matrix
from geometry_msgs.msg._PoseStamped import PoseStamped

def get_angle(p1, p2):
    '''
    Calculates the angle between two points.
    :param p1: Point
    :param p2: Point
    :return: angle as float
    '''
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return np.arccos(dot_product(v1,v2) / (magnitude(v1) * magnitude(v2)))


def three_points_to_quaternion(origin, to, roll=None):
    '''
    Calculates a quaternion that points from "origin" to "to" and lies in the plane defined by "origin", "to" and "roll".
    :param origin: Point
    :param to: Point
    :param roll: Point
    :return: Quaternion
    '''
    muh = False
    if roll is None:
        roll = Point(0,0,origin.z+1.000001)
        muh = True
    n_1 = subtract_point(to, origin)
    n_1 = normalize(n_1)

    n = subtract_point(roll, origin)
    n = normalize(n)

    n_2 = subtract_point(n, multiply_point(dot_product(n, n_1), n_1))
    n_2 = normalize(n_2)

    n_3 = cross_product(n_1, n_2)
    n_3 = normalize(n_3)

    rm = np.array(((n_1.x, n_2.x, n_3.x, 0),
                      (n_1.y, n_2.y, n_3.y, 0),
                      (n_1.z, n_2.z, n_3.z, 0),
                      (0, 0, 0, 1)), dtype=np.float64)

    q = quaternion_from_matrix(rm)
    q = Quaternion(*q)
    if muh :
        q = rotate_quaternion(q, pi/2, 0, 0)
    return q


def subtract_point(p1, p2):
    '''
    p1 - p2
    :param p1: Point / array
    :param p2: Point / array
    :return: Point
    '''
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return Point(*np.subtract(v1, v2))

def euclidean_distance(p1, p2):
    return magnitude(subtract_point(p1, p2))

def add_point(p1, p2):
    '''
    p1 + p2
    :param p1: Point / array
    :param p2: Point / array
    :return: Point
    '''
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return Point(*np.add(v1, v2))


def normalize(p):
    '''
    Normalizes a point.
    :param p: Point / array
    :return: Point
    '''
    v = p
    if type(p) is Point:
        v = (p.x, p.y, p.z)

    l = magnitude(v)
    result = Point(*(x * 1/l for x in v))

    return result

def set_vector_length(l, p):
    '''
    Sets the length of "p" to "l"
    :param l: float
    :param p: Point / array
    :return:Point
    '''
    return multiply_point(l, normalize(p))

def multiply_point(s, p):
    '''
    p * s
    :param s: float
    :param p: Point / array
    :return: Point
    '''
    v = p
    if type(p) is Point:
        v = (p.x, p.y, p.z)
    return Point(*np.multiply(v, s))


def dot_product(p1, p2):
    '''
    p1 * p2
    :param p1: Point / array
    :param p2: Point / array
    :return: scalar product as Point
    '''
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return np.dot(v1, v2)


def magnitude(q):
    '''
    Calculates the length of a quaternion of point
    :param q: Quaternion/Point
    :return: lenght as float
    '''
    v = q
    if type(q) is Point:
        v = (q.x, q.y, q.z)
    if type(q) is geometry_msgs.msg.Quaternion:
        v = (q.x, q.y, q.z, q.w)
    return np.linalg.norm(v)

def cross_product(p1, p2):
    '''
    p1 x p2
    :param p1: Point / array
    :param p2: Point / array
    :return: Point
    '''
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return Point(*np.cross(v1, v2))

def rotate_quaternion(q, roll, pitch, yaw):
    '''
    Rotates a quaternion by roll, pitch, yaw.
    :param q: Quaternion
    :param r: float
    :param p: float
    :param y: float
    :return: Quaternion
    '''
    angle = quaternion_from_euler(roll, pitch, yaw)
    no = quaternion_multiply([q.x, q.y, q.z, q.w], angle)
    return Quaternion(*no)

def euler_to_quaternion(roll, pitch, yaw):
    '''
    Creates a quaternion out of roll, pitch, yaw
    :param roll: float
    :param pitch: float
    :param yaw: float
    :return: Quaternion
    '''
    return Quaternion(*quaternion_from_euler(roll, pitch, yaw))

def get_pitch(q):
    '''
    Calculates the pitch of a quaternion.
    :param q: Quaternion
    :return: float
    '''
    gripper_direction = qv_mult(q, Point(1, 0, 0))
    v = deepcopy(gripper_direction)

    v.z = 0
    if 0 <= magnitude(v) <= 0.001:
        v = Point(1, 0, 0)
    pitch = get_angle(v, gripper_direction)
    return pitch



def normalize2(v, tolerance=0.00001):
    '''
    Normalize with tolerance.
    :param v: (float, float, float, float)/ (float, float, float)
    :param tolerance: float
    :return: (float, float, float, float)/ (float, float, float)
    '''
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def qv_mult(q1, v1):
    '''
    Transforms a vector by a quaternion
    :param q1: Quaternion
    :param v1: Point
    :return: Point
    '''
    q = q1
    v = v1
    if type(q1) is Quaternion:
        q = (q1.x, q1.y, q1.z, q1.w)
    if type(v1) is Point:
        v = (v1.x, v1.y, v1.z, 0)

    r = quaternion_multiply(quaternion_multiply(q, v), quaternion_conjugate(q))
    return Point(r[0], r[1], r[2])


def orientation_to_vector(orientation):
    '''
    Transforms a 1, 0, 0 vector by a quaternion
    :param orientation: Quaternion
    :return: Point
    '''
    return qv_mult(orientation, Point(1, 0, 0))

