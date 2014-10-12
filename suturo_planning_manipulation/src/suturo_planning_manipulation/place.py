from copy import deepcopy
from numpy.lib.function_base import place
from scipy.stats.distributions import planck_gen
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
import rospy
import shape_msgs
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from tf.transformations import quaternion_from_euler
from math import sqrt, pi, cos, sin, acos
from calc_grasp_position import finger_length, hand_length, pre_grasp_length, get_angle, magnitude, rotate_quaternion, \
    subtract_point, multiply_point, set_vector_length, make_scan_pose
from mathemagie import *
from manipulation_constants import *

__author__ = 'ichumuh'


def get_place_position(collision_object, dest, tf_listener, transform_func, d, grasp=None):
    if len(collision_object.primitives) == 1:
        if collision_object.primitives[0].type == SolidPrimitive.BOX:
            return get_place_position_cube(collision_object, dest, grasp,d)
        else:
            return get_place_position_cube(collision_object, dest, grasp,d)
    else:
        return get_place_position_handle(collision_object, dest, tf_listener, grasp, transform_func, d)


def get_place_position_cube(collision_object, dest, grasp,d):
    angle = get_pitch(grasp)
    print "angle", angle
    z_o = abs(grasp.pose.position.z) - (sin(angle) * d)

    place_pose = dest.point
    place_pose.z = z_o + safe_place
    diff = abs(pi/2 - angle)
    if 0 <= diff <= 0.1:
        return make_scan_pose(place_pose, d, angle, n=1)
    else:
        return make_scan_pose(place_pose, d, angle)

def get_place_position_handle(collision_object, dest, tf_listener, grasp, transform_func, d):

    angle = 0

    place_pose = dest.point

    now = rospy.Time.now()
    tf_listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
    (p, egal) = tf_listener.lookupTransform("/tcp", "/" + collision_object.id, now)

    if 0 <= abs(p[1]) < 0.01:
        place_pose.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + \
                                     collision_object.primitives[1].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] + safe_place
    else:
        place_pose.z = collision_object.primitives[0].dimensions[
                                     shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] + \
                                 collision_object.primitives[1].dimensions[
                                     shape_msgs.msg.SolidPrimitive.BOX_X] + \
                                 collision_object.primitives[2].dimensions[
                                     shape_msgs.msg.SolidPrimitive.BOX_X] / 2 + safe_place

    place_poses = make_scan_pose(place_pose, d, angle)
    if p[1] > 0:
        for i in range(0, len(place_poses)):
            place_poses[i].pose.orientation = rotate_quaternion(place_poses[i].pose.orientation, pi, 0, 0)

    return place_poses


def get_grasped_part(collision_object, transform_func):
    print collision_object
    co = transform_func(collision_object, "/link7")
    print co
    b = -1
    posi = Point()
    i = 0
    id = 0
    for cop in co.primitive_poses:
        a = (abs(cop.position.x) + abs(cop.position.y)) / 2
        if b < 0 or a < b:
            b = a
            posi = cop.position
            id = i
        i += 1
    print id , " pose ", posi , " co ", collision_object.primitives[id].type
    return (posi, id)


def get_pre_place_position(place_pose):
    pre_place_pose = deepcopy(place_pose)
    pre_place_pose.pose.position.z += pre_place_length
    return pre_place_pose

