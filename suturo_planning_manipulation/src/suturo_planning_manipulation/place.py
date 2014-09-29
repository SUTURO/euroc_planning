from copy import deepcopy
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
import rospy
import shape_msgs
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from tf.transformations import quaternion_from_euler
from math import sqrt, pi, cos, sin, acos
from calc_grasp_position import finger_length, hand_length, pre_grasp_length, get_angle, magnitude, rotate_quaternion, \
    subtract_point, multiply_point, set_vector_length

__author__ = 'ichumuh'

from manipulation_constants import *


def get_place_position(collision_object, dest, tf_listener, transform_func, grasp=None):
    dest2 = set_place_pose_x_y(dest, collision_object, transform_func, tf_listener, grasp)
    if len(collision_object.primitives) == 1:
        if collision_object.primitives[0].type == SolidPrimitive.BOX:
            return get_place_position_cube(collision_object, dest2, grasp, tf_listener)
        else:
            return get_place_position_cylinder(collision_object, dest2, grasp, tf_listener)
    else:
        return get_place_position_handle(collision_object, dest, tf_listener, grasp, transform_func)


def set_place_pose_x_y(dest, collision_object, transform_func, tf_listener, grasp=None):
    g = deepcopy(grasp)
    g.z = 0
    m = magnitude(g)


    place_pose = PoseStamped()
    alpha = get_pitch(grasp)
    q = quaternion_from_euler(0, alpha, get_yaw(dest.point))
    place_pose.pose.orientation = Quaternion(*q)
    place_pose.header = dest.header
    place_pose.pose.position = set_vector_length(sqrt(dest.point.x**2 + dest.point.y**2) - m, dest.point)
    return place_pose


def get_place_position_cube(collision_object, dest, grasp, tf_listener):
    place_pose = dest

    b = max(collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X],
            collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y],
            collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z])
    place_pose.pose.position.z = abs(grasp.z) + b/2 + safe_place

    return place_pose


def get_place_position_cylinder(collision_object, dest, grasp, tf_listener):
    place_pose = dest

    place_pose.pose.position.z = abs(grasp.z) + collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + safe_place

    return place_pose


def get_place_position_handle(collision_object, dest, tf_listener, grasp, transform_func):
    # TODO: besser machen
    g = deepcopy(grasp)
    # g.z = 0
    # m = magnitude(g)


    place_pose = PoseStamped()
    alpha = get_pitch(grasp)
    # q = quaternion_from_euler(0, alpha, get_yaw(dest.point))
    # place_pose.pose.orientation = Quaternion(*q)
    place_pose.header = dest.header
    place_pose.pose.position = set_vector_length(sqrt(dest.point.x**2 + dest.point.y**2) - abs(g.z), dest.point)



    p = PoseStamped()
    p.header.frame_id = "/odom_combined"
    p.pose.orientation.w = 1
    p.pose.position = get_grasped_part(collision_object)

    p = transform_func(p, "/tcp")


    # place_pose.pose.orientation = rotate_quaternion(place_pose.pose.orientation, 0, -pi/2, 0)

    # place_pose.header = dest.header

    # place_pose.pose.position.y = dest.point.y

    now = rospy.Time.now()
    tf_listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
    (p, q) = tf_listener.lookupTransform("/tcp", "/" + collision_object.id, now)

    a = max(p)


    # place_pose.pose.position.x -= a

    # box1 = tf_listener()

    if 0 <= abs(p[1]) < 0.01:
        q = quaternion_from_euler(0, 0, get_yaw(dest.point))
        place_pose.pose.orientation = Quaternion(*q)
        place_pose.pose.position.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + \
                                     collision_object.primitives[1].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] + safe_place
    elif p[1] > 0:
        q = quaternion_from_euler(pi, 0, get_yaw(dest.point))
        place_pose.pose.orientation = Quaternion(*q)
        place_pose.pose.position.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] + \
                                     collision_object.primitives[1].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] + \
                                     collision_object.primitives[2].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] / 2 + safe_place
    else:
        q = quaternion_from_euler(0, 0, get_yaw(dest.point))
        place_pose.pose.orientation = Quaternion(*q)
        place_pose.pose.position.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] + \
                                     collision_object.primitives[1].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] + \
                                     collision_object.primitives[2].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] / 2 +safe_place


    # print place_pose.pose.position
    # place_pose.pose.position.z = finger_length + hand_length \
    # + collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2
    return place_pose


def get_grasped_part(collision_object):
    b = -1
    posi = Point()
    for cop in collision_object.primitive_poses:
        a = abs(cop.position.x) + abs(cop.position.y) / 2
        if b < 0 or a < b:
            b = a
            posi = cop.position
    return posi


def get_pre_place_position(place_pose):
    pre_place_pose = deepcopy(place_pose)
    pre_place_pose.pose.position.z += pre_place_length
    return pre_place_pose


def get_pitch(g):
    # g = subtract_point(obj, grasp)
    v1 = Point()
    v1.x = g.x
    v1.y = g.y
    if magnitude(v1) == 0:
        return pi / 2
    # v2 = deepcopy(g)
    # v2 = subtract_point(v2, obj)
    pitch = get_angle(v1, g)

    # if grasp is None:
    #     return pi / 2
    # v1 = Point()
    # v1.x = grasp.x
    # v1.y = grasp.y
    # if magnitude(v1) == 0:
    #     return pi / 2
    # v2 = deepcopy(grasp)
    # v2 = subtract_point(v2, obj)
    # pitch = get_angle(v1, v2)
    return pitch


def get_yaw(dest):
    x = Point(1, 0, 0)

    d = dest
    d.z = 0

    alpha = get_angle(x, d)
    if d.y < 0:
        alpha = -alpha
    return alpha