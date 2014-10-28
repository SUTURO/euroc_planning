#!/usr/bin/env python
from asyncore import dispatcher
from docutils.parsers.rst.roles import role
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
from place import get_place_position, get_pre_place_position, pre_place_length, post_place_length, get_grasped_part
from planningsceneinterface import *
from manipulation_constants import *
from manipulation_service import *
import math
# from suturo_planning_visualization.visualization import visualize_poses
from suturo_planning_visualization.visualization import visualize_poses
from transformer import Transformer
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from sensor_msgs.msg import JointState


class Manipulation(object):

    def __init__(self):
        self.tf = Transformer()

        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm_group = moveit_commander.MoveGroupCommander("arm")
        self.__arm_group.set_planning_time(5)

        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.__base_group = moveit_commander.MoveGroupCommander("base")
        self.__base_group.set_planning_time(2.5)

        self.__arm_base_group = moveit_commander.MoveGroupCommander("arm_base")
        self.__arm_base_group.set_planning_time(10)

        rospy.wait_for_service('/euroc_interface_node/move_along_joint_path')
        self.__service = rospy.ServiceProxy('/euroc_interface_node/move_along_joint_path', MoveAlongJointPath)

        self.__planning_scene_interface = PlanningSceneInterface()

        euroc_interface_node = '/euroc_interface_node/'
        self.__set_object_load_srv = rospy.ServiceProxy(euroc_interface_node + 'set_object_load', SetObjectLoad)

        rospy.sleep(1)
        self.__planning_scene_interface.add_ground()
        self.__planning_scene_interface.add_cam_mast()

        self.__grasp = None

        self.__manService = ManipulationService()

        rospy.loginfo( "Manipulation started.")

    def __del__(self):
        if moveit_commander is not None:
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

    def move_base(self, goal_pose):
        '''
        Moves the arm's base to the goal position. (Don't use this for Task 1 and 2)
        :param goal_pose: goal position as a PoseStamped
        :return: success of the movement
        '''
        goal = [goal_pose.pose.position.x, goal_pose.pose.position.y]
        m = self.__base_group.get_current_joint_values()
        d1 = goal[0] - m[0]
        d2 = goal[1] - m[1]
        if 0 <= abs(d1) <= 0.01 and 0 <= abs(d2) <= 0.01:
            rospy.loginfo("No movement required.")
            return True
        self.__base_group.set_joint_value_target(goal)
        path = self.__base_group.plan()
        return self.__manService.move(path)

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        '''
        Transforms the pose_target into the target_frame.
        :param pose_target: object to transform as PoseStamped/PointStamped/Vector3Stamped/CollisionObject/PointCloud2
        :param target_frame: goal frame id
        :return: transformed object
        '''
        return self.tf.transform_to(pose_target, target_frame)

    def move_to(self, goal_pose,blow_up = True):
        '''
        Moves the endeffector to the goal position, without moving the base.
        :param goal_pose: goal position as PoseStamped
        :return: success of the movement
        '''
        return self.__move_group_to(goal_pose, self.__arm_group, blow_up = blow_up)

    def move_arm_and_base_to(self, goal_pose, blow_up = True):
        '''
        Moves the endeffector to the goal position. (Don't use this for Task 1 and 2)
        :param goal_pose: goal position as PoseStamped
        :return: success of the movement
        '''
        return self.__move_group_to(goal_pose, self.__arm_base_group, blow_up = blow_up)

    def get_base_origin(self):
        '''
        :return: The centre of the arm's base as PointStamped
        '''
        current_pose = self.__base_group.get_current_joint_values()
        result = PointStamped()
        result.header.frame_id = "/odom_combined"
        result.point = Point(current_pose[0], current_pose[1], 0)
        return result

    def get_eef_position(self):
        '''
        :return: The centre of the arm's base as PointStamped
        '''
        current_pose = self.__arm_group.get_current_pose()
        return current_pose

    def __blow_up_object(self, bobject, factor):
       """
        :param bobject: Object to blow up
        :param factor: Blowup Factor
        :return: Returns the Blown up object
        """
       for primitive in bobject.primitives:
           dims = []
           for dimension in primitive.dimensions:
               dims.append(dimension + factor)
           primitive.dimensions = dims
       return bobject

    def __move_group_to(self, goal_pose, move_group, blow_up=True, blow_up_distance = 0.02):
        """
         :param goal_pose: the pose which shall be arrived
         :param move_group: the move group which shall be moved
         :param blow_up: True if collision objects shall be made bigger
         :param blow_up_distance: Distance in m
         :return:
         """

        original_objects = self.__planning_scene_interface.get_collision_objects()
        blown_up_objects = []
        if blow_up:
            for each in original_objects:
                if not each.id in self.__planning_scene_interface.safe_objects:
                    # self.__planning_scene_interface.remove_object(each.id)
                    bobj = self.__blow_up_object(copy.deepcopy(each), blow_up_distance)
                    blown_up_objects.append(bobj.id)
                    self.__planning_scene_interface.add_object(bobj)
        move_group.set_start_state_to_current_state()
        goal = deepcopy(goal_pose)
        if type(goal) is str:
            move_group.set_named_target(goal)
        elif type(goal) is PoseStamped:
            visualize_poses([goal])
            #Rotate the goal so that the gripper points from 0,0,0 to 1,0,0 with a 0,0,0,1 quaternion as orientation.
            goal.pose.orientation = rotate_quaternion(goal.pose.orientation, pi/2, pi, pi/2)
            goal = self.tf.transform_to(goal)

            move_group.set_pose_target(goal)
        else:
            move_group.set_joint_value_target(goal)

        path = move_group.plan()

        if blow_up:
            # for each in blown_up_objects:
            #     self.__planning_scene_interface.remove_object(each)
            for each in original_objects:
                self.__planning_scene_interface.add_object(each)

        return self.__manService.move(path)

    def get_current_joint_state(self):
        '''
        :return: current joint state as list of floats
        '''
        return self.__arm_base_group.get_current_joint_values()

    def get_current_lwr_joint_state(self):
        print self.__arm_group.get_joints()
        return self.__arm_group.get_current_joint_values()

    def open_gripper(self, position=gripper_max_pose):
        '''
        Opens the gripper and detaches any attached collisionobject.
        :param position: the desired finger position, max value if not specified.
        :return: success of the movement
        '''
        self.__gripper_group.set_joint_value_target([-position, position])
        path = self.__gripper_group.plan()
        if self.__manService.move(path):
            self.__gripper_group.detach_object()
            self.load_object(0, Vector3(0, 0, 0))
            return True
        else:
            return False

    def close_gripper(self, object=None):
        '''
        Closes the gripper completely or far enough to hold the object, when one is given
        :param object: Object that will be grasped.
        :return: success of the movement
        '''
        if type(object) is CollisionObject:
            self.__gripper_group.attach_object(object.id, "gp", ["gp", "finger1", "finger2"])
            rospy.sleep(1.0)
            # (egal, id) = get_grasped_part(object, self.tf.transform_to)
            id = 0
            #TODO: only works for cubes and cylinders and only "sometimes" for object compositions
            if object.primitives[id].type == shape_msgs.msg.SolidPrimitive.BOX:
                length = min(object.primitives[id].dimensions)
                self.__gripper_group.set_joint_value_target([-(length/2), length/2])
            elif object.primitives[id].type == shape_msgs.msg.SolidPrimitive.CYLINDER:
                radius = object.primitives[id].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]
                self.__gripper_group.set_joint_value_target([-radius+0.005, radius-0.005])
        else:
            self.__gripper_group.set_joint_value_target([0.0, 0.0])
        path = self.__gripper_group.plan()
        return self.__manService.move(path)

    def grasp(self, collision_object, object_density=1):
        '''
        Deprecated. For testing only
        '''
        return self.__grasp_with_group(collision_object, self.__arm_group, object_density)

    def grasp_and_move(self, collision_object, object_density=1):
        '''
        Deprecated. For testing only
        '''
        return self.__grasp_with_group(collision_object, self.__arm_base_group, object_density)

    def __grasp_with_group(self, collision_object_name, move_group, object_density):
        '''
        Deprecated. For testing only
        '''
        if type(collision_object_name) is CollisionObject:
            collision_object_name = collision_object_name.id

        collision_object = self.__planning_scene_interface.get_collision_object(collision_object_name)
        if collision_object is None:
            rospy.logwarn("Collision Object " + collision_object_name + " is not in planningscene.")
            return False

        grasp_positions = calculate_grasp_position(collision_object, self.tf.transform_to)


        grasp_positions = self.filter_invalid_grasps(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found for " + collision_object_name)

        grasp_positions.sort(cmp=lambda x, y: self.cmp_pose_stamped(collision_object, x, y))
        # visualize_poses(grasp_positions)
        # print grasp_positions

        self.open_gripper()
        for grasp in grasp_positions:
            if self.__move_group_to(get_pre_grasp(grasp), move_group):

                if not self.__move_group_to(grasp, move_group):
                    continue
                rospy.sleep(1)
                self.close_gripper(collision_object)

                com = self.get_center_of_mass(collision_object)
                com = self.tf.transform_to(com, "/tcp")
                if com is None:
                    rospy.logwarn("TF failed")
                    return False
                self.load_object(self.calc_object_weight(collision_object, object_density), Vector3(com.point.x, com.point.y, com.point.z))

                rospy.loginfo("grasped " + collision_object_name)


                self.__grasp = self.tf.transform_to(grasp)
                v1 = deepcopy(self.__grasp.pose.position)
                v1.z = 0
                v2 = deepcopy(collision_object.primitive_poses[0].position)
                v2.z = 0
                a = magnitude(subtract_point(v1, v2))
                b = abs(self.__grasp.pose.position.z - collision_object.primitive_poses[0].position.z)
                c = sqrt(a**2 + b**2)
                self.__d = abs(c)
                print c

                rospy.logdebug("lift object")
                if not self.__move_group_to(get_pre_grasp(grasp), move_group):
                    rospy.logdebug("couldnt lift object")
                return True
        rospy.logwarn("Grapsing failed.")
        return False

    def cmp_pose_stamped(self, collision_object, pose1, pose2):
        '''
        Compares tow poses by calculating the distance to the centre of a collision object and
        returns -1/0/1 depending on which on is closer.
        :param collision_object: collision object as CollisionObject
        :param pose1: first pose as PoseStamped
        :param pose2: second pose as PoseStamped
        :return: pose1 > pose2
        '''
        #TODO:richtigen abstand zum center berechnen
        center = self.get_center_of_mass(collision_object)
        odom_pose1 = self.tf.transform_to(pose1)
        odom_pose2 = self.tf.transform_to(pose2)
        d1 = magnitude(subtract_point(center.point, odom_pose1.pose.position))
        d2 = magnitude(subtract_point(center.point, odom_pose2.pose.position))
        diff = d1 - d2
        if 0.0 <= abs(diff) <= 0.015 or len(collision_object.primitives) == 1:
            z1 = odom_pose1.pose.position.z
            z2 = odom_pose2.pose.position.z
            diff = z2 - z1
        return 1 if diff > 0 else -1 if diff < 0 else 0

    def filter_invalid_grasps(self, list_of_grasps):
        '''
        Filters out positions that are very close to the ground.
        :param list_of_grasps: list of PoseStamped
        :return: filtered list of PoseStamped
        '''
        #TODO: assumes odom_combined as frame id
        if len(list_of_grasps) == 0:
            return list_of_grasps

        return filter(lambda x : self.tf.transform_to(x).pose.position.z > min_grasp_height, list_of_grasps)

    def calc_object_weight(self, collision_object, density):
        '''
        Calculates the weight of a collision object with the given density
        :param collision_object: CollisionObject
        :param density: density as float
        :return: weight as float
        '''
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
        '''
        Calculates the centre of a collision object.
        :param collision_object: CollisionObject
        :return: centre of mass as PointStamped
        '''
        p = PointStamped()
        p.header.frame_id = "/odom_combined"
        for pose in collision_object.primitive_poses:
            p.point = add_point(p.point, pose.position)
        p.point = multiply_point(1.0 / len(collision_object.primitive_poses), p.point)

        return p

    def place(self, destination):
        '''
        Deprecated. For testing only
        '''
        return self.__place_with_group(destination, self.__arm_group)

    def place_and_move(self, destination):
        '''
        Deprecated. For testing only
        '''
        return self.__place_with_group(destination, self.__arm_base_group)

    def __place_with_group(self, destination, move_group):
        '''
        Deprecated. For testing only
        '''
        dest = deepcopy(destination)
        # print dest
        co = self.__planning_scene_interface.get_attached_object()
        if co is None:
            return False
        else:
            co = co.object
        dest = self.tf.transform_to(dest)
        place_poses = get_place_position(co, dest, self.tf.transform_to, self.__d, self.__grasp)
        # visualize_poses(place_poses)
        for place_pose in place_poses:
            if not self.__move_group_to(get_pre_place_position(place_pose), move_group):
                rospy.logwarn("Can't reach preplaceposition.")
                continue
            if not self.__move_group_to(place_pose, move_group):
                rospy.logwarn("Can't reach placeposition.")
                continue

            rospy.sleep(1)
            if not self.open_gripper():
                return False
            rospy.sleep(1)

            post_place_pose = self.tf.transform_to(place_pose, co.id)
            # post_place_pose.header.frame_id = "/tcp"
            # post_place_pose.pose.position = Point(0, 0, -post_place_length)

            if not self.__move_group_to(get_pre_grasp(post_place_pose), move_group):
                rospy.logwarn("Can't reach postplaceposition.")
                return True
            rospy.sleep(0.25)
            rospy.loginfo("placed " + co.id)
            return True
        return False

    def load_object(self, mass, cog):
        '''
        Tells euroc that and object has been grasped.
        :param mass: mass of the object as float
        :param cog: centre of mass as Vector3
        :return: response message
        '''
        request = SetObjectLoadRequest()
        request.mass = mass
        request.center_of_gravity = cog
        resp = self.__set_object_load_srv(request)
        # print resp.error_message
        return resp

    def get_planning_scene(self):
        return self.__planning_scene_interface

    def turn_arm(self, joint_value):
        """
        Sets "link1" to "joint_value
        :param joint_value: float #radian -2.96 to 2.96
        :return: success of the movement
        """
        current_joint_values = self.__arm_group.get_current_joint_values()
        current_joint_values[0] = joint_value
        self.__arm_group.set_joint_value_target(current_joint_values)
        path = self.__arm_group.plan()
        return self.__manService.move(path)

    def get_arm_move_group(self):
        return self.__arm_group

    def get_arm_base_move_group(self):
        return self.__arm_base_group

    # def set_height_constraint(self, t=True):
    #     if t:
    #         self.__planning_scene_interface.add_ground(0.95)
    #     else:
    #         self.__planning_scene_interface.remove_object("ground0.95")

    def pan_tilt(self, pan, tilt):
        '''
        Moves the scene cam.
        :param pan: desired pan as float
        :param tilt: desired tilt as float
        :return: success of the movement
        '''
        return self.__manService.pan_tilt(pan, tilt)

    def direct_move(self, configuration):
        return self.__manService.direct_move(configuration)