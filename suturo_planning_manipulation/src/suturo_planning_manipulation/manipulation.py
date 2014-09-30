#!/usr/bin/env python
from math import pi
from pdb import post_mortem

import sys
import copy
from euroc_c2_msgs.srv import *
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._Vector3 import Vector3
from geometry_msgs.msg._Vector3Stamped import Vector3Stamped
from moveit_commander import planning_scene_interface
from moveit_msgs.msg import *
from numpy.core.multiarray import ndarray
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from calc_grasp_position import *
from place import get_place_position, get_pre_place_position, pre_place_length, post_place_length
from planningsceneinterface import *
from manipulation_constants import *
import math

class Manipulation(object):
    def __init__(self):
        self.__listener = tf.TransformListener()

        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm_group = moveit_commander.MoveGroupCommander("arm")
        self.__arm_group.set_planning_time(5)

        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.__base_group = moveit_commander.MoveGroupCommander("base")
        self.__base_group.set_planning_time(15)

        self.__arm_base_group = moveit_commander.MoveGroupCommander("arm_base")
        self.__arm_base_group.set_planning_time(15)

        self.__planning_scene_interface = PlanningSceneInterface()

        euroc_interface_node = '/euroc_interface_node/'
        self.__set_object_load_srv = rospy.ServiceProxy(euroc_interface_node + 'set_object_load', SetObjectLoad)

        rospy.sleep(1)
        self.__planning_scene_interface.add_ground()
        # self.set_height_constraint(True)

        self.__grasp = None
        rospy.loginfo( "Manipulation started.")

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move_base(self, goal_pose):
        goal = deepcopy(goal_pose)
        self.__base_group.set_joint_value_target([goal.pose.position.x, goal.pose.position.y])
        # print goal
        r = self.__base_group.go()
        rospy.loginfo("moved base")
        return r

    def move_to(self, goal_pose):
        return self.__move_group_to(goal_pose, self.__arm_group)

    def move_arm_and_base_to(self, goal_pose):
        return self.__move_group_to(goal_pose, self.__arm_base_group)

    def __move_group_to(self, goal_pose, move_group):
        move_group.set_start_state_to_current_state()
        goal = deepcopy(goal_pose)
        if type(goal) is str:
            move_group.set_named_target(goal)
        else:
            visualize_pose([goal])

            goal.pose.orientation = rotate_quaternion(goal.pose.orientation, pi/2, 0, pi/2)

            goal = self.transform_to(goal)

            move_group.set_pose_target(goal)

        return move_group.go()

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        odom_pose = None
        i = 0
        while odom_pose is None and i < 10:
            try:
                if type(pose_target) is CollisionObject:
                    i = 0
                    new_co = deepcopy(pose_target)
                    for cop in pose_target.primitive_poses:
                        p = PoseStamped()
                        p.header = pose_target.header
                        p.pose.orientation = cop.orientation
                        p.pose.position = cop.position
                        p = self.transform_to(p, target_frame)
                        if p is None:
                            return None
                        new_co.primitive_poses[i].position = p.pose.position
                        new_co.primitive_poses[i].orientation = p.pose.orientation
                        i += 1
                    new_co.header.frame_id = target_frame
                    return new_co
                if type(pose_target) is PoseStamped:
                    odom_pose = self.__listener.transformPose(target_frame, pose_target)
                    break
                if type(pose_target) is Vector3Stamped:
                    odom_pose = self.__listener.transformVector3(target_frame, pose_target)
                    break
                if type(pose_target) is PointStamped:
                    odom_pose = self.__listener.transformPoint(target_frame, pose_target)
                    break
            except Exception, e:
                print "tf error:::", e
            rospy.sleep(0.5)
            # pose_target.header.stamp = rospy.Time.now()
            # self.__listener.waitForTransform(target_frame, pose_target.header.frame_id, pose_target.header.stamp, rospy.Duration(4))
            i += 1
            print pose_target
            rospy.logdebug("tf fail nr. " + str(i))

        if odom_pose is None:
            rospy.logerr("FUUUUUUUUUUUUUU!!!! fucking tf shit!!!!")
        return odom_pose

    # def tf_lookup(self, frame1, frame2):
    #
    #     i = 0
    #     while i < 10:
    #         try:
    #             now = rospy.Time.now()
    #
    #             self.__listener.waitForTransform("/odom_combined", "/tcp", now, rospy.Duration(4))
    #             (p, q) = self.__listener.lookupTransform("/odom_combined", "/tcp", now)
    #
    #             self.__listener.waitForTransform("/odom_combined", "/" + object_name, now, rospy.Duration(4))
    #             (p2, q2) = self.__listener.lookupTransform("/odom_combined", "/" + object_name, now)
    #
    #         except Exception, e:
    #             print "tf error:::", e
    #         rospy.sleep(0.5)
    #         i += 1
    #         rospy.logdebug("tf fail nr. " + str(i))
    #
    #     return odom_pose


    def open_gripper(self, position=gripper_max_pose):
        self.__gripper_group.set_joint_value_target([-position, position])
        if not self.__gripper_group.go():
            rospy.logwarn("Failed to open gripper.")
            return False
        self.__gripper_group.detach_object()

        self.load_object(0, Vector3(0, 0, 0))
        return True

    def close_gripper(self, object=None):
        #TODO:fix sonderfall bei komposition
        if type(object) is CollisionObject:
            if object.id != "":
                self.__gripper_group.attach_object(object.id, "gp", ["gp", "finger1", "finger2"])
                rospy.sleep(0.5)
            if object.primitives[0].type == 1:
                length = object.primitives[0].dimensions[0]
                self.__gripper_group.set_joint_value_target([-(length/2), length/2])
            elif object.primitives[0].type == 3:
                radius = object.primitives[0].dimensions[1]
                self.__gripper_group.set_joint_value_target([-radius+0.005, radius-0.005])
        else:
            self.__gripper_group.set_joint_value_target([0.0, 0.0])
        self.__gripper_group.go()

    def grasp(self, collision_object, object_density=1):
        return self.__grasp_with_group(collision_object, self.__arm_group, object_density)

    def grasp_and_move(self, collision_object, object_density=1):
        return self.__grasp_with_group(collision_object, self.__arm_base_group, object_density)

    def __grasp_with_group(self, collision_object_name, move_group, object_density):
        if type(collision_object_name) is CollisionObject:
            collision_object_name = collision_object_name.id

        collision_object = self.__planning_scene_interface.get_collision_object(collision_object_name)
        if collision_object is None:
            rospy.logwarn("Collision Object " + collision_object_name + " is not in planningscene.")
            return False

        grasp_positions = calculate_grasp_position(collision_object, self.transform_to)


        grasp_positions = self.__filter_invalid_grasps(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found for " + collision_object_name)

        grasp_positions.sort(cmp=lambda x, y: self.cmp_pose_stamped(collision_object, x, y))
        # visualize_pose(grasp_positions)
        # print grasp_positions

        self.open_gripper()
        for grasp in grasp_positions:
            if self.__move_group_to(get_pre_grasp(grasp), move_group):

                if not self.__move_group_to(grasp, move_group):
                    continue
                rospy.sleep(1)
                self.close_gripper(collision_object)

                com = self.get_center_of_mass(collision_object)
                com = self.transform_to(com, "/tcp")
                if com is None:
                    rospy.logwarn("TF failed")
                    return False
                self.load_object(self.calc_object_weight(collision_object, object_density), Vector3(com.point.x, com.point.y, com.point.z))

                rospy.loginfo("grasped " + collision_object_name)
                self.__grasp = self.make_grasp_vector(collision_object_name)

                rospy.logdebug("lift object")
                if not self.__move_group_to(get_pre_grasp(grasp), move_group):
                    rospy.logdebug("couldnt lift object")
                return True
        rospy.logwarn("Grapsing failed.")
        return False

    def make_grasp_vector(self, object_name):
        now = rospy.Time.now()

        self.__listener.waitForTransform("/odom_combined", "/tcp", now, rospy.Duration(4))
        (p, q) = self.__listener.lookupTransform("/odom_combined", "/tcp", now)

        self.__listener.waitForTransform("/odom_combined", "/" + object_name, now, rospy.Duration(4))
        (p2, q2) = self.__listener.lookupTransform("/odom_combined", "/" + object_name, now)

        g = subtract_point(Point(*p2), Point(*p))
        # print g
        return g

    def cmp_pose_stamped(self, collision_object, pose1, pose2):
        #TODO:richtigen abstand zum center berechnen
        center = self.get_center_of_mass(collision_object)
        odom_pose1 = self.transform_to(pose1)
        odom_pose2 = self.transform_to(pose2)
        d1 = magnitude(subtract_point(center.point, odom_pose1.pose.position))
        d2 = magnitude(subtract_point(center.point, odom_pose2.pose.position))
        diff = d1 - d2
        if 0.0 <= abs(diff) <= 0.015 or len(collision_object.primitives) == 1:
            z1 = odom_pose1.pose.position.z
            z2 = odom_pose2.pose.position.z
            diff = z2 - z1
        return 1 if diff > 0 else -1 if diff < 0 else 0

    def __filter_invalid_grasps(self, list_of_grasps):
        if len(list_of_grasps) == 0:
            return list_of_grasps

        return filter(lambda x : self.transform_to(x).pose.position.z > min_grasp_height, list_of_grasps)

    def calc_object_weight(self, collision_object, density):
        weight = 0
        for i in range(0, len(collision_object.primitives)):
            if collision_object.primitives[i].type == shape_msgs.msg.SolidPrimitive().BOX:
                x = collision_object.primitives[i].dimensions[shape_msgs.msg.SolidPrimitive.BOX_X]
                y = collision_object.primitives[i].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Y]
                z = collision_object.primitives[i].dimensions[shape_msgs.msg.SolidPrimitive.BOX_Z]
                weight += x * y * z * density
            elif collision_object.primitives[i].type == shape_msgs.msg.SolidPrimitive().CYLINDER:
                r = collision_object.primitives[i].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]
                h = collision_object.primitives[i].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_HEIGHT]
                weight += pi * r * r * h * density
        return weight

    def get_center_of_mass(self, collision_object):
        p = PointStamped()
        p.header.frame_id = "/odom_combined"
        for pose in collision_object.primitive_poses:
            p.point = add_point(p.point, pose.position)
        p.point = multiply_point(1.0 / len(collision_object.primitive_poses), p.point)

        return p

    def place(self, destination):
        """ destination of type pose-stamped """
        return self.__place_with_group(destination, self.__arm_group)

    def place_and_move(self, destination):
        """ destination of type pose-stamped """
        return self.__place_with_group(destination, self.__arm_base_group)

    def __place_with_group(self, destination, move_group):
        """ destination of type pose-stamped """
        dest = deepcopy(destination)
        print dest
        co = self.__planning_scene_interface.get_attached_object()
        if co is None:
            return False
        else:
            co = co.object
        dest = self.transform_to(dest)
        place_pose = get_place_position(co, dest, self.__listener, self.transform_to, self.__grasp)
        if not self.__move_group_to(get_pre_place_position(place_pose), move_group):
            rospy.logwarn("Can't reach preplaceposition.")
            return False
        if not self.__move_group_to(place_pose, move_group):
            rospy.logwarn("Can't reach placeposition.")
            return False

        rospy.sleep(1)
        if not self.open_gripper():
            return False
        rospy.sleep(1)

        post_place_pose = PoseStamped()
        post_place_pose.header.frame_id = "/tcp"
        post_place_pose.pose.position = Point(0, 0, -post_place_length)

        if not self.__move_group_to(post_place_pose, move_group):
            rospy.logwarn("Can't reach postplaceposition.")
            return False
        rospy.sleep(0.25)
        rospy.loginfo("placed " + co.id)
        return True

    def load_object(self, mass, cog):
        request = SetObjectLoadRequest()
        request.mass = mass
        request.center_of_gravity = cog
        resp = self.__set_object_load_srv(request)
        print resp.error_message
        return resp

    def get_planning_scene(self):
        return self.__planning_scene_interface

    def turn_arm(self, joint_value):
        """
        :param joint_value: float #radian -2.96 to 2.96
        :return: undefined
        """
        current_joint_values = self.__arm_group.get_current_joint_values()
        current_joint_values[0] = joint_value
        self.__arm_group.set_joint_value_target(current_joint_values)
        return self.__arm_group.go()

    def get_arm_move_group(self):
        return self.__arm_group

    def set_height_constraint(self, t=True):
        if t:
            self.__planning_scene_interface.add_ground(0.95)
        else:
            self.__planning_scene_interface.remove_object("ground0.95")

    # Arguments: geometry_msgs/PointStamped, double distance from point to camera, double camera angle
    def object_cam_pose(self, point, distance, angle):
        # Get the data!
        alpha = angle
        dist = distance
        object = point

        # Get x and y point from object
        point_x = object.pose.position.x
        point_y = object.pose.position.y
        # get sin_beta
        #if point_x == 0:
        #    sin_beta = 0
        #else:
        sin_beta = (point_y / point_x)
        # get diagonal from the middle of the table to the object
        v = math.sqrt((point_x**2) + (point_y**2))
        # initialize cam_pose and roll objects
        cam_pose = geometry_msgs.msg.PoseStamped()
        roll = geometry_msgs.msg.PoseStamped()

        cam_pose.header.frame_id = point.header.frame_id
        cam_pose.pose.orientation = point.pose.orientation

        # calculate the distance from the object to the desired cam_pose
        w = math.cos(alpha) * dist
        #if point_x == 0:
        #    beta = 0
        #else:
        beta = math.atan(point_y / point_x)
        # calculate x, y and z value from the cam pose
        cam_x = (v - w) * math.cos(beta)
        cam_y = cam_x * sin_beta
        cam_z = dist * math.sin(alpha)
        # set this values...
        cam_pose.pose.position.x = cam_x
        cam_pose.pose.position.y = cam_y
        cam_pose.pose.position.z = cam_z

        x1 = -point_x
        y1 = -point_y
        x2 = 1
        y2 = -x1 / y1

        roll.pose.position.x = x2
        roll.pose.position.y = y2
        roll.pose.position.z = object.pose.position.z

        # calculate the quaternion
        quaternion = three_points_to_quaternion(cam_pose.pose.position, object.pose.position, roll.pose.position)

        cam_pose.pose.orientation = quaternion

        self.move_to(cam_pose)
        
        return cam_pose
