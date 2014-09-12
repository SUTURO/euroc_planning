#!/usr/bin/env python
from math import pi
from pdb import post_mortem

import sys
import copy
from euroc_c2_msgs.srv import *
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Vector3 import Vector3
from geometry_msgs.msg._Vector3Stamped import Vector3Stamped
from numpy.core.multiarray import ndarray
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from calc_grasp_position import *
from place import get_place_position, get_pre_place_position, pre_place_length, post_place_length
from planningsceneinterface import *

gripper_max_pose = 0.03495


class Manipulation(object):

    def __init__(self):
        self.__listener = tf.TransformListener()
        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm_group = moveit_commander.MoveGroupCommander("arm")
        self.__arm_group.set_planning_time(5)
        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.__planning_scene_interface = PlanningSceneInterface()
        self.__base_group = moveit_commander.MoveGroupCommander("base")

        euroc_interface_node = '/euroc_interface_node/'
        self.__set_object_load_srv = rospy.ServiceProxy(euroc_interface_node + 'set_object_load', SetObjectLoad)
        # self.__gripper_max_pose = 0.03495
        rospy.sleep(2)
        self.__planning_scene_interface.add_ground()
        # self.__arm_group.set_path_constraints()

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def move_base(self, goal_pose):
        #print self.__base_group.get_current_pose()
        goal = deepcopy(goal_pose)
        #print goal
        self.__base_group.get_join
        self.__base_group.set_joint_value_target([goal.pose.position.x,goal.pose.position.y])
        return self.__base_group.go()

    def move_to(self, goal_pose):
        goal = deepcopy(goal_pose)
        if type(goal) is str:
            self.__arm_group.set_named_target("scan_pose1")
        else:
            visualize_pose([goal])
            angle = quaternion_from_euler(0, pi / 2, 0)
            o = goal.pose.orientation
            no = quaternion_multiply([o.x, o.y, o.z, o.w], angle)
            goal.pose.orientation = geometry_msgs.msg.Quaternion(*no)

            goal = self.transform_to(goal)
            self.__arm_group.set_pose_target(goal)

        return self.__arm_group.go()

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        odom_pose = None
        i = 0
        while odom_pose is None and i < 5:
            try:
                if type(pose_target) is PoseStamped:
                    # self.__listener.waitForTransform()
                    odom_pose = self.__listener.transformPose(target_frame, pose_target)
                    break
                if type(pose_target) is Vector3Stamped:
                    odom_pose = self.__listener.transformVector3(target_frame, pose_target)
                    break
                if type(pose_target) is PointStamped:
                    odom_pose = self.__listener.transformPoint(target_frame, pose_target)
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "tf error"
            rospy.sleep(0.5)
            i += 1
            print "tf fail nr. ", i

        if odom_pose is None:
            print "FUUUUUUUUUUUUUU!!!! fucking tf shit!!!!"
        return odom_pose

    def open_gripper(self, position=gripper_max_pose):
        self.__gripper_group.set_joint_value_target([-position, position])
        self.__gripper_group.go()
        self.__gripper_group.detach_object()

        self.load_object(0, Vector3(0, 0, 0))

    def close_gripper(self, object_name=""):
        if object_name.id != "":
            self.__gripper_group.attach_object(object_name.id, "gp", ["gp", "finger1", "finger2"])
            rospy.sleep(0.5)
        if object_name.primitives[0].type == 1:
            length = object_name.primitives[0].dimensions[0]
            self.__gripper_group.set_joint_value_target([-(length/2), length/2])
        if object_name.primitives[0].type == 3:
            radius = object_name.primitives[0].dimensions[1]
            self.__gripper_group.set_joint_value_target([-radius+0.005, radius-0.005])
        self.__gripper_group.go()

    def grasp(self, collision_object):
        if type(collision_object) is str:
            collision_object = self.__planning_scene_interface.get_collision_object(collision_object)
        grasp_positions = calculate_grasp_position(collision_object)
        self.sort_grasps(grasp_positions)
        print len(grasp_positions)
        self.open_gripper()
        for grasp in grasp_positions:
            if self.move_to(get_pre_grasp(grasp)):

                if not self.move_to(grasp):
                    continue
                rospy.sleep(1)
                self.close_gripper(collision_object)

                self.load_object(1, self.get_center_of_mass(collision_object))
                print "grasped"
                # rospy.sleep(1)
                return True
        return None

    def get_center_of_mass(self, collision_object):
        # center_of_mass= Vector3Stamped()
        # point = Point()
        # if len(collision_object.primitives) == 1:
        #     point = collision_object.primitive_poses[0].position
        #
        # center_of_mass.header.frame_id = collision_object.header.frame_id
        # center_of_mass.vector = Vector3(point.x, point.y, point.z)
        # print "center:  ", center_of_mass
        # center_of_mass = self.transform_to(center_of_mass, "link7")
        # print "center2:  ", center_of_mass

        now = rospy.Time.now()
        self.__listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(4))
        (p, q) = self.__listener.lookupTransform("/tcp", "/" + collision_object.id, now)

        return Vector3(p[0], p[1], p[2])

    def stop(self):
        self.__arm_group.stop()

    def place(self, destination):
        """ destination of type pose-stamped """
        dest = deepcopy(destination)
        co = self.__planning_scene_interface.get_attached_object().object
        dest = self.transform_to(dest, "/odom_combined")
        place_pose = get_place_position(co, dest, self.__listener)
        self.move_to(get_pre_place_position(place_pose))
        self.move_to(place_pose)
        self.open_gripper()
        rospy.sleep(0.25)

        post_place_pose = PoseStamped()
        post_place_pose.header.frame_id = "/tcp"
        post_place_pose.pose.position = Point(0, 0, -post_place_length)
        self.move_to(post_place_pose)
        rospy.sleep(0.25)

    def load_object(self, mass, cog):
        request = SetObjectLoadRequest()
        request.mass = mass
        request.center_of_gravity = cog
        resp = self.__set_object_load_srv(request)
        print resp.error_message
        return resp

    def sort_grasps(self, grasps):
        grasps.sort(key=self.__grasp_value)

    def __grasp_value(self, grasp):
        return -self.transform_to(grasp).pose.position.z

    def get_planning_scene(self):
        return self.__planning_scene_interface

    def turn_arm(self, speed, distance, linkid=0):
        """
        :param speed: float
        :param distance: float #radian 0 - 5,9341
        :return: undefined
        """
        # distance -= 2.96705972839 #-joint limit
        jv = self.__arm_group.get_current_joint_values()
        jv[linkid] = distance
        self.__arm_group.set_joint_value_target(jv)
        return self.__arm_group.go()

    def get_arm_move_gourp(self):
        return self.__arm_group