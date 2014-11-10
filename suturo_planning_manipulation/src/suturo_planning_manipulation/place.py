from copy import deepcopy
from numpy.lib.function_base import place
from scipy.stats.distributions import planck_gen
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from moveit_msgs.msg._CollisionObject import CollisionObject
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


def get_place_position(collision_object, destination, transform_func, d, grasp):
    '''
    Calculates place position for a "collision_object" at "dest".
    :param collision_object: CollisionObject
    :param destination: PointStamped
    :param transform_func(object, frame_id): a function to transform objects to different frame_ids. (use Manipulation.transform_to)
    :param d: dist from the grasped point to the tcp frame
    :param grasp: PoseStamped
    :return: list of PoseStamped
    '''
    if len(collision_object.primitives) == 1:
        return get_place_position_for_single_object(collision_object, destination, transform_func, grasp,d)
    else:
        return get_place_position_handle(collision_object, destination, transform_func, d)


def get_place_position_for_puzzle(destination):
    rospy.logdebug("get_place_position_for_puzzle")
    place_poses = []
    origin = deepcopy(destination.point)
    to = add_point(deepcopy(destination.point), Point(0, 0, -1))
    roll = add_point(deepcopy(destination.point), Point(1, 0, 0))

    place_pose_1 = PoseStamped()
    place_pose_1.header.frame_id = "/odom_combined"
    place_pose_1.pose.position = deepcopy(destination.point)
    place_pose_1.pose.orientation = three_points_to_quaternion(deepcopy(origin), deepcopy(to), deepcopy(roll))
    place_poses.append(place_pose_1)

    roll = add_point(deepcopy(destination.point),Point(-1, 0, 0))
    place_pose_2 = deepcopy(place_pose_1)
    place_pose_2.pose.orientation = three_points_to_quaternion(deepcopy(origin), deepcopy(to), deepcopy(roll))
    place_poses.append(place_pose_2)

    roll = add_point(deepcopy(destination.point),Point(0, 1, 0))
    place_pose_3 = deepcopy(place_pose_1)
    place_pose_3.pose.orientation = three_points_to_quaternion(deepcopy(origin), deepcopy(to), deepcopy(roll))
    place_poses.append(place_pose_3)

    roll = add_point(deepcopy(destination.point),Point(0, -1, 0))
    place_pose_4 = deepcopy(place_pose_1)
    place_pose_4.pose.orientation = three_points_to_quaternion(deepcopy(origin), deepcopy(to), deepcopy(roll))
    place_poses.append(place_pose_4)

    return place_poses

def get_place_position_for_single_object(collision_object, destination, transform_func,  grasp, d):
    '''
    Calculates place positions for a non composition of collision objects.
    :param collision_object: CollisionObject
    :param destination: PointStamped
    :param transform_func(object, frame_id): a function to transform objects to different frame_ids. (use Manipulation.transform_to)
    :param grasp: PoseStamped
    :param d: dist from the grasped point to the tcp frame
    :return: list of PoseStamped
    '''
    angle = get_pitch(grasp.pose.orientation)
    z_o = abs(grasp.pose.position.z) - (sin(angle) * d)

    place_pose = destination.point
    place_pose.z = z_o + safe_place
    diff = abs(pi/2 - angle)
    if 0 <= diff <= 0.1:
        place_poses = make_scan_pose(place_pose, d, angle, n=2)
    else:
        place_poses = make_scan_pose(place_pose, d, angle)

    p2 = PointStamped()
    p2.header.frame_id = collision_object.id
    p2 = transform_func(p2, "tcp")
    rospy.logwarn(p2)
    if p2.point.y < 0:
        for i in range(0, len(place_poses)):
            place_poses[i].pose.orientation = rotate_quaternion(place_poses[i].pose.orientation, pi, 0, 0)

    return place_poses

def get_place_position_handle(collision_object, destination, transform_func, d):
    '''
    Calculates place positions for a composition of collision objects.
    :param collision_object: CollisionObject
    :param destination: PointStamped
    :param transform_func(object, frame_id): a function to transform objects to different frame_ids. (use Manipulation.transform_to)
    :param d: dist from the grasped point to the tcp frame
    :return: list of PoseStamped
    '''
    angle = 0

    place_pose = destination.point

    p2 = PointStamped()
    p2.header.frame_id = collision_object.id
    p2 = transform_func(p2, "tcp")

    place_pose.z = collision_object.primitives[0].dimensions[
                                         shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT] / 2 + \
                                     collision_object.primitives[1].dimensions[
                                         shape_msgs.msg.SolidPrimitive.BOX_X] + safe_place
    place_pose.z += abs(p2.point.y)

    place_poses = make_scan_pose(place_pose, d, angle)
    if p2.point.y < 0:
        for i in range(0, len(place_poses)):
            place_poses[i].pose.orientation = rotate_quaternion(place_poses[i].pose.orientation, pi, 0, 0)

    return place_poses

# def point_inBounding_box(object, point):
#     o = CollisionObject()
#     bounding_box = Point()
#     for p in object.primitives:
#         if p.type == SolidPrimitive.CYLINDER:
#             bounding_box.x = p.dimensions[SolidPrimitive.CYLINDER_RADIUS]*2
#             bounding_box.y = p.dimensions[SolidPrimitive.CYLINDER_RADIUS]*2
#             bounding_box.z = p.dimensions[SolidPrimitive.CYLINDER_HEIGHT]
#         else:
#             bounding_box.x = p.dimensions[SolidPrimitive.BOX_X]
#             bounding_box.y = p.dimensions[SolidPrimitive.BOX_Y]
#             bounding_box.z = p.dimensions[SolidPrimitive.BOX_Z]

    # if


def get_grasped_part(collision_object, grasped_point):
    """
    Returns the grasped part of a collision object
    :param collision_object: CollisionObject
    :param transform_func(object, frame_id): a function to transform objects to different frame_ids. (use Manipulation.transform_to)
    :return: (Point, float), grasped point and id of the grasped part
    """
    #TODO: buggy, use bounding box instead of closed object centroid
    if grasped_point is None:
        return (None, 0)

    for i in range(len(collision_object.primitive_poses)):
        print euclidean_distance(collision_object.primitive_poses[i].position, grasped_point.point)

    id = min(range(len(collision_object.primitive_poses)), key=lambda copid: euclidean_distance(collision_object.primitive_poses[copid].position, grasped_point.point))
    return (collision_object.primitive_poses[id], id)

def get_pre_place_position(place_pose):
    '''
    Returns a position that is higher than the place position, should be taken before placing.
    :param place_pose: PoseStamped
    :return: PoseStamped
    '''
    pre_place_pose = deepcopy(place_pose)
    pre_place_pose.pose.position.z += pre_place_length
    return pre_place_pose

