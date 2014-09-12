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

gripper_max_pose = 0.03495


class Manipulation(object):
    def __init__(self):
        self.__listener = tf.TransformListener()
        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm_group = moveit_commander.MoveGroupCommander("arm")
        self.__arm_group.set_planning_time(10)
        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.__planning_scene_interface = PlanningSceneInterface()
        self.__base_group = moveit_commander.MoveGroupCommander("base")
        self.__arm_base_group = moveit_commander.MoveGroupCommander("arm_base")

        euroc_interface_node = '/euroc_interface_node/'
        self.__set_object_load_srv = rospy.ServiceProxy(euroc_interface_node + 'set_object_load', SetObjectLoad)
        # self.__gripper_max_pose = 0.03495
        rospy.sleep(1)
        self.__planning_scene_interface.add_ground()
        print "manipulation started"
        # self.__arm_group.set_path_constraints()
        # self.set_constraint()
        # self.__arm_group.set_planner_id("RRTConnectkConfigDefault")


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

    def move_to(self, goal_pose, constraint=False):
        self.__arm_group.set_start_state_to_current_state()
        goal = deepcopy(goal_pose)
        if type(goal) is str:
            self.__arm_group.set_named_target("scan_pose1")
        else:
            visualize_pose([goal])
            angle = quaternion_from_euler(0, pi / 2, 0)
            o = goal.pose.orientation
            no = quaternion_multiply([o.x, o.y, o.z, o.w], angle)
            goal.pose.orientation = geometry_msgs.msg.Quaternion(*no)

            if constraint:
                self.set_constraint(goal)

            print goal

            goal = self.transform_to(goal)
            self.__arm_group.set_pose_target(goal)

        return self.__arm_group.go()

    def move_arm_and_base(self, goal_pose):
        goal = deepcopy(goal_pose)
        if type(goal) is str:
            self.__arm_base_group.set_named_target("scan_pose1")
        else:
            visualize_pose([goal])
            angle = quaternion_from_euler(0, pi / 2, 0)
            o = goal.pose.orientation
            no = quaternion_multiply([o.x, o.y, o.z, o.w], angle)
            goal.pose.orientation = geometry_msgs.msg.Quaternion(*no)

            goal = self.transform_to(goal)
            self.__arm_base_group.set_pose_target(goal)

        return self.__arm_base_group.go()

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        odom_pose = None
        i = 0
        while odom_pose is None and i < 5:
            try:
                if type(pose_target) is PoseStamped:
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

    def grasp(self, collision_object, object_density=1):
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
                self.close_gripper(collision_object.id)

                self.load_object(self.calc_object_weight(collision_object, object_density),
                                 self.get_center_of_mass(collision_object))
                print "grasped"
                # rospy.sleep(1)
                return True
        return None

    def grasp_and_move(self, collision_object):
        if type(collision_object) is str:
            collision_object = self.__planning_scene_interface.get_collision_object(collision_object)
        grasp_positions = calculate_grasp_position(collision_object)
        self.sort_grasps(grasp_positions)
        print len(grasp_positions)
        self.open_gripper()
        for grasp in grasp_positions:
            if self.move_arm_and_base(get_pre_grasp(grasp)):

                if not self.move_arm_and_base(grasp):
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

    def calc_object_weight(self, collision_object, density):
        weight = 0
        for i in range(0, len(collision_object.primitives)):
            print i
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
        now = rospy.Time.now()
        self.__listener.waitForTransform("/tcp", "/" + collision_object.id, now, rospy.Duration(5))
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

    def place_and_move(self, destination):
        """ destination of type pose-stamped """
        dest = deepcopy(destination)
        co = self.__planning_scene_interface.get_attached_object().object
        dest = self.transform_to(dest, "/odom_combined")
        place_pose = get_place_position(co, dest, self.__listener)
        self.move_arm_and_base(get_pre_place_position(place_pose))
        self.move_arm_and_base(place_pose)
        self.open_gripper()
        rospy.sleep(0.25)

        post_place_pose = PoseStamped()
        post_place_pose.header.frame_id = "/tcp"
        post_place_pose.pose.position = Point(0, 0, -post_place_length)
        self.move_arm_and_base(post_place_pose)
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

    def set_constraint(self, pose):
        c = Constraints()
        # c.name = ""
        # pc = PositionConstraint()
        # pc.header.frame_id = "/odom_combined"
        # pc.link_name = "gp"
        # box = SolidPrimitive()
        # box.type = SolidPrimitive.SPHERE
        # box.dimensions.append(2.0)
        # pc.constraint_region.primitives.append(box)
        # box_pose = Pose()
        # box_pose.position = Point(0.0, 0.0, 1.0)
        # box_pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        # pc.constraint_region.primitive_poses.append(box_pose)
        # pc.weight = 1.0
        # c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.link_name = "link7"
        oc.weight = 1.0
        oc.header = pose.header
        oc.orientation = pose.pose.orientation
        # oc.orientation = Quaternion(0, 0, 0, 1)
        oc.absolute_x_axis_tolerance = pi
        oc.absolute_y_axis_tolerance = pi
        oc.absolute_z_axis_tolerance = 0.1  # pi
        c.orientation_constraints.append(oc)
        print c
        self.__arm_group.set_path_constraints(c)
