from copy import deepcopy
from math import pi
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
import rospy
import shape_msgs
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from tf.transformations import quaternion_from_euler
from calc_grasp_position import finger_length, hand_length, pre_grasp_length

__author__ = 'ichumuh'

pre_place_length = 0.05
safe_place = 0.005


def get_place_position(collision_object, dest, tf_listener):
    if len(collision_object.primitives) == 1:
        if collision_object.primitives[0].type == SolidPrimitive.BOX:
            return get_place_position_cube(collision_object, dest, tf_listener)
        else:
            return get_place_position_cylinder(collision_object, dest, tf_listener)
    else:
        return get_place_position_handle(collision_object, dest, tf_listener)


def get_place_position_cube(collision_object, dest, tf_listener):
    place_pose = PoseStamped()
    q = quaternion_from_euler(0, pi / 2, 0)
    place_pose.pose.orientation = Quaternion(*q)
    place_pose.header = dest.header
    place_pose.pose.position.x = dest.point.x
    place_pose.pose.position.y = dest.point.y

    now = rospy.Time.now()
    tf_listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
    (p, q) = tf_listener.lookupTransform("/tcp", "/" + collision_object.id, now)

    a = max(p)
    print "p ", p

    print "dimensions: ", collision_object.primitives[0].dimensions
    place_pose.pose.position.z = a + collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] / 2 + safe_place

    print "dest: ", place_pose.pose.position
    # place_pose.pose.position.z = finger_length + hand_length \
    # + collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X] / 2
    return place_pose


def get_place_position_cylinder(collision_object, dest, tf_listener):
    place_pose = PoseStamped()
    q = quaternion_from_euler(0, pi / 2, 0)
    place_pose.pose.orientation = Quaternion(*q)
    place_pose.header = dest.header
    place_pose.pose.position.x = dest.point.x
    place_pose.pose.position.y = dest.point.y

    now = rospy.Time.now()
    tf_listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
    (p, q) = tf_listener.lookupTransform("/tcp", "/" + collision_object.id, now)

    a = max(p)

    place_pose.pose.position.z = a + collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + safe_place
    print place_pose.pose.position
    # place_pose.pose.position.z = finger_length + hand_length \
    # + collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2
    return place_pose


def get_place_position_handle(collision_object, dest, tf_listener):
    place_pose = PoseStamped()
    q = quaternion_from_euler(pi/2, 0, 0)
    place_pose.pose.orientation = Quaternion(*q)
    place_pose.header = dest.header

    place_pose.pose.position.y = dest.point.y

    now = rospy.Time.now()
    tf_listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
    (p, q) = tf_listener.lookupTransform("/tcp", "/" + collision_object.id, now)

    a = max(p)

    place_pose.pose.position.x = dest.point.x - a

    place_pose.pose.position.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + \
                                 collision_object.primitives[1].dimensions[
                                     shape_msgs.msg.SolidPrimitive.BOX_X] + safe_place
    print place_pose.pose.position
    # place_pose.pose.position.z = finger_length + hand_length \
    # + collision_object.primitives[0].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2
    return place_pose


def get_pre_place_position(place_pose):
    pre_place_pose = deepcopy(place_pose)
    pre_place_pose.pose.position.z += pre_place_length
    return pre_place_pose