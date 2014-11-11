from geometry_msgs.msg._PointStamped import PointStamped
from numpy.core.multiarray import dot
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from suturo_msgs.msg._Object import Object
from suturo_planning_manipulation.manipulation_constants import hand_length, finger_length
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
    """
    Calculates the angle between two points.
    :param p1: first point
    :type: Point
    :param p2: second point
    :type: Point
    :return: angle
    :type: float
    """
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    a = dot_product(v1,v2) / (magnitude(v1) * magnitude(v2))
    #because fuck u python
    if a >= 0.9999999:
        a = 0.9999999
    elif a <= -0.9999999:
        a = -0.9999999
    return np.arccos(a)


def three_points_to_quaternion(origin, to, roll=None):
    """
    Calculates a quaternion that points from "origin" to "to" and lies in the plane defined by "origin", "to" and "roll".
    :param origin: form this point
    :type: Point
    :param to: to this point
    :type: Point
    :param roll: in a plane defined by this third point
    :type: Point or None
    :return: orientation
    :type: Quaternion
    """
    muh = False
    if roll is None: #TODO buggy
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
        q = rotate_quaternion(q, -pi/2, 0, 0)
    return q


def get_fingertip(hand_pose):
    """
    Calculates the point between the finger tips, where objects will be hold.
    :param hand_pose: hand pose
    :type: PoseStamped
    :return: point between fingers
    :type: PointStamped
    """
    grasp_point = PointStamped()
    grasp_point.point = set_vector_length(hand_length + finger_length, Point(1,0,0))
    grasp_point.point = qv_mult(hand_pose.pose.orientation, grasp_point.point)
    grasp_point.point = add_point(hand_pose.pose.position , grasp_point.point)
    grasp_point.header.frame_id = hand_pose.header.frame_id
    return grasp_point


def subtract_point(p1, p2):
    """
    p1 - p2
    :param p1: first point
    :type: Point/ (float(x), float(y), float(z))
    :param p2: second point
    :type: Point/ (float(x), float(y), float(z))
    :return: Point
    :type: Point
    """
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return Point(*np.subtract(v1, v2))

def euclidean_distance(p1, p2):
    """
    Calculates the euclidean distance between two points.
    :param p1: first point
    :type: Point
    :param p2: second point
    :type: Point
    :return: distance
    :type: float
    """
    return magnitude(subtract_point(p1, p2))

def euclidean_distance_in_2d(p1, p2):
    """
    Calculates the euclidean distance between two points, but ignores the z value.
    :param p1: first point
    :type: Point
    :param p2: second point
    :type: Point
    :return: distance
    :type: float
    """
    p = deepcopy(p2)
    p.z = 0
    return magnitude(subtract_point(p1, p))

def add_point(p1, p2):
    """
    p1 + p2
    :param p1: first point
    :type: Point/ (float(x), float(y), float(z))
    :param p2: second point
    :type: Point/ (float(x), float(y), float(z))
    :return: Point
    :type: Point
    """
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return Point(*np.add(v1, v2))


def calc_object_volume(object):
    """
    Calculates the Volume of an object
    :param object: object
    :type: CollisionObject / Object
    :return: volume
    :type: float
    """
    volume = 0
    for i in xrange(len(object.primitives)):
        if object.primitives[i].type == SolidPrimitive().BOX:
            x = object.primitives[i].dimensions[SolidPrimitive.BOX_X]
            y = object.primitives[i].dimensions[SolidPrimitive.BOX_Y]
            z = object.primitives[i].dimensions[SolidPrimitive.BOX_Z]
            volume += x * y * z
        elif object.primitives[i].type == SolidPrimitive().CYLINDER:
            r = object.primitives[i].dimensions[SolidPrimitive.CYLINDER_RADIUS]
            h = object.primitives[i].dimensions[SolidPrimitive.CYLINDER_HEIGHT]
            volume += pi * r * r * h
    return volume


def normalize(p):
    """
    Normalizes a point.
    :param p: point
    :type: Point / (float(x), float(y), float(z))
    :return: normalized point
    :type: Point
    """
    v = p
    if type(p) is Point:
        v = (p.x, p.y, p.z)

    l = magnitude(v)
    result = Point(*(x * 1/l for x in v))

    return result

def set_vector_length(l, p):
    """
    Sets the length of "p" to "l"
    :param l: desired length
    :type: float
    :param p: point
    :type: Point / (float(x), float(y), float(z))
    :return: point with desired length
    :type: Point
    """
    return multiply_point(l, normalize(p))

def multiply_point(s, p):
    """
    p * s
    :param s: factor
    :type: float
    :param p: point
    :type: Point / (float(x), float(y), float(z))
    :return: multiplied point
    :type: Point
    """
    v = p
    if type(p) is Point:
        v = (p.x, p.y, p.z)
    return Point(*np.multiply(v, s))


def dot_product(p1, p2):
    """
    p1 * p2
    :param p1: first point
    :type: Point / (float(x), float(y), float(z))
    :param p2: second point
    :type: Point / (float(x), float(y), float(z))
    :return: scalar product
    :type: float
    """
    v1 = p1
    if type(p1) is Point:
        v1 = (p1.x, p1.y, p1.z)
    v2 = p2
    if type(p2) is Point:
        v2 = (p2.x, p2.y, p2.z)
    return np.dot(v1, v2)


def magnitude(q):
    """
    Calculates the length of a quaternion of point
    :param q: Quaternion/Point
    :type: Point / Quaternion
    :return: length
    :type: float
    """
    v = q
    if type(q) is Point:
        v = (q.x, q.y, q.z)
    if type(q) is geometry_msgs.msg.Quaternion:
        v = (q.x, q.y, q.z, q.w)
    return np.linalg.norm(v)

def cross_product(p1, p2):
    """
    p1 x p2
    :param p1: first point
    :type: Point / (float(x), float(y), float(z))
    :param p2: second point
    :type: Point / (float(x), float(y), float(z))
    :return: cross product
    :type: Point
    """
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
    :type: Quaternion
    :param roll: roll
    :type: float
    :param pitch: pitch
    :type: float
    :param yaw: yaw
    :type: float
    :return: rotated quaternion
    :type: Quaternion
    '''
    angle = quaternion_from_euler(roll, pitch, yaw)
    no = quaternion_multiply([q.x, q.y, q.z, q.w], angle)
    return Quaternion(*no)

def rotate_quaternion_by_quaternion(q1, q2):
    """
    Rotates a quaternion by another quaternion
    :param q1: first Quaternion
    :type: Quaternion
    :param q2: second Quaternion
    :type: Quaternion
    :return: rotated Quaternion
    :type: Quaternion
    """
    r = quaternion_multiply([q1.x, q1.y, q1.z, q1.w], [q2.x, q2.y, q2.z, q2.w])
    return Quaternion(*r)

def euler_to_quaternion(roll, pitch, yaw):
    """
    Creates a quaternion out of roll, pitch, yaw
    :param roll:
    :type: float
    :param pitch:
    :type: float
    :param yaw:
    :type: float
    :return: Quaternion from roll pitch yaw
    :type: Quaternion
    """
    return Quaternion(*quaternion_from_euler(roll, pitch, yaw))

def get_pitch(q):
    """
    Calculates the pitch of a quaternion.
    :param q: Quaternion
    :type: Quaternion
    :return: pitch of the quaternion
    :type: float
    """
    gripper_direction = qv_mult(q, Point(1, 0, 0))
    v = deepcopy(gripper_direction)

    v.z = 0
    if 0 <= magnitude(v) <= 0.001:
        v = Point(1, 0, 0)
    pitch = get_angle(v, gripper_direction)
    return pitch

def get_yaw(q):
    """
    Calculates the pitch of a quaternion.
    :param q: Quaternion
    :type: Quaternion
    :return: yaw of the quaternion
    :type: float
    """
    gripper_direction = qv_mult(q, Point(1, 0, 0))
    v = deepcopy(gripper_direction)

    v.z = 0
    if 0 <= magnitude(v) <= 0.001:
        v = Point(1, 0, 0)
    yaw = get_angle(v, Point(1, 0, 0))
    return yaw

def normalize2(v, tolerance=0.00001):
    """
    Normalize with tolerance.
    :param v: vector/ quaternion
    :type: (float, float, float, float)/ (float, float, float)
    :param tolerance: tolerance
    :type: float
    :return: normalized vector / quaternion
    :type: (float, float, float, float)/ (float, float, float)
    """
    mag2 = sum(n * n for n in v)
    if abs(mag2 - 1.0) > tolerance:
        mag = sqrt(mag2)
        v = tuple(n / mag for n in v)
    return v


def qv_mult(q1, v1):
    """
    Transforms a vector by a quaternion
    :param q1: Quaternion
    :type: Quaternion
    :param v1: vector
    :type: Point
    :return: transformed vector
    :type: Point
    """
    q = q1
    v = v1
    if type(q1) is Quaternion:
        q = (q1.x, q1.y, q1.z, q1.w)
    if type(v1) is Point:
        v = (v1.x, v1.y, v1.z, 0)

    r = quaternion_multiply(quaternion_multiply(q, v), quaternion_conjugate(q))
    return Point(r[0], r[1], r[2])


def orientation_to_vector(orientation):
    """
    Transforms a 1, 0, 0 vector by a quaternion, the "roll" information is lost.
    :param orientation: orientation
    :type: Quaternion
    :return: vector
    :type: Point

    """
    return qv_mult(orientation, Point(1, 0, 0))


def get_puzzle_fixture_center(puzzle_fixture):
    '''
    Calculates the center of the puzzle fixture. Assumed fixture dimensions: 32cm x 32cm x 10cm
    :param puzzle_fixture: geometry_msgs/Pose
    :return: Point
    '''
    l = sqrt(2*0.30*0.30) / 2
    v1 = orientation_to_vector(rotate_quaternion(deepcopy(puzzle_fixture.orientation), 0, 0, pi/4))
    p1 = puzzle_fixture.position
    v1norm = normalize(v1)
    print("v1 = "+str(v1))
    print("v1norm = "+str(v1norm))
    p2 = Point(v1norm.x * l, v1norm.y * l, v1norm.z * l)
    print("p2 len = " + str(sqrt(p2.x*p2.x + p2.y*p2.y)))
    return add_point(p1, p2)
