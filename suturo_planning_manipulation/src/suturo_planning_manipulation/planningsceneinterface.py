#!/usr/bin/env python

__author__ = 'ichumuh'

from __builtin__ import staticmethod
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
import moveit_commander
import moveit_msgs
from moveit_msgs.msg._CollisionObject import CollisionObject
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
import moveit_msgs.msg
import moveit_msgs.srv
from calc_grasp_position import *


class PlanningSceneInterface(object):
    safe_objects = []
    def __init__(self):
        self.__collision_object_publisher = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject,
                                                            queue_size=10)
        self.__attached_object_publisher = rospy.Publisher('/attached_collision_object',
                                                           moveit_msgs.msg.AttachedCollisionObject,
                                                           queue_size=10)
        rospy.wait_for_service("get_planning_scene")
        self.__ps_service_client = rospy.ServiceProxy('get_planning_scene', moveit_msgs.srv.GetPlanningScene)
        rospy.sleep(0.1)

    def __del__(self):
        pass

    def add_ground(self, height=-0.01):
        '''
        Adds a big flat box to the planning scene.
        :param height: float, z value
        '''
        pose = PoseStamped()
        pose.header.frame_id = "/odom_combined"
        pose.pose.position = Point(0, 0, height)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        box = self.make_box("ground" + str(height), pose, [3, 3, 0.01])
        self.safe_objects.append(box.id)
        self.add_object(box)

    def add_cam_mast(self):
        '''
        Adds a Cylinder where the cam mast is.
        '''
        pose = PoseStamped()
        pose.header.frame_id = "/odom_combined"
        pose.pose.position = Point(0.92, 0.92, 0.55)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        c1 = self.make_cylinder("cm1", pose, [1.1, 0.05])
        self.safe_objects.append(c1.id)
        self.add_object(c1)

        pose.pose.position = Point(0.92, 0.92, 1.0)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        c1 = self.make_box("mast_cams", pose, [0.3, 0.3, 0.3])
        self.safe_objects.append(c1.id)
        self.add_object(c1)

    @staticmethod
    def make_box(name, pose, size):
        '''
        Creates a box collision object.
        :param name: str, name of the box
        :param pose: PoseStamped, position of the box
        :param size: list of float
        :return: CollisionObject
        '''
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(size)
        co.primitives = [box]
        co.primitive_poses = [pose.pose]
        return co

    @staticmethod
    def make_cylinder(name, pose, size):
        '''
        Creates a cylinder collision object.
        :param name: str, name of the cylinder
        :param pose: PoseStamped, position of the cylinder
        :param size: list of float
        :return: CollisionObject
        '''
        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = name
        co.header = pose.header
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions = list(size)
        co.primitives = [cylinder]
        co.primitive_poses = [pose.pose]
        return co

    def get_collision_objects(self):
        '''
        :return: list of all CollisionObjects, that are currently in the planning scene
        '''
        return self.get_planning_scene().world.collision_objects

    def get_collision_object(self, name):
        '''
        Returns a collision object with the given name out of the planning scene.
        :param name: str
        :return: CollisionObject
        '''
        cos = self.get_collision_objects()
        for co in cos:
            if co.id == name:
                return co
        return None

    def get_attached_objects(self):
        '''
        :return: list of attached CollisionObject
        '''
        return self.get_planning_scene().robot_state.attached_collision_objects

    def get_attached_object(self):
        '''
        :return: the attached CollisionObject
        '''
        acos = self.get_attached_objects()
        if len(acos) == 0:
            rospy.logerr("No objects attached!")
            return None
        else:
            return acos[0]

    def get_planning_scene(self):
        '''
        :return: the PlanningScene
        '''
        try:
            ps = self.__ps_service_client(moveit_msgs.msg.PlanningSceneComponents(1023))
            return ps.scene

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            return None

    def add_object(self, collision_object):
        '''
        Adds a collision object to the planning scene.
        :param collision_object: CollisionObject
        '''
        collision_object.operation = moveit_msgs.msg.CollisionObject.ADD
        self.__collision_object_publisher.publish(collision_object)
        rospy.sleep(0.1)

    def add_objects(self, collision_objects):
        '''
        Adds a list of collision objects to the planning scene.
        :param collision_objects: list of CollisionObject
        '''
        for co in collision_objects:
            co.operation = moveit_msgs.msg.CollisionObject.ADD
            self.__collision_object_publisher.publish(co)
            rospy.sleep(0.01)

    def add_box(self, name, pose, size):
        '''
        Adds a box to the planning scene.
        :param name: str
        :param pose: PoseStamped
        :param size: list of float
        '''
        self.add_object(self.make_box(name, pose, size))

    def remove_object(self, name):
        '''
        Removes a collision object from the planning scene.
        :param name: str, name of the collision object
        '''
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.REMOVE
        co.id = name
        self.__collision_object_publisher.publish(co)

        aco = moveit_msgs.msg.AttachedCollisionObject()
        aco.object = co
        aco.link_name = "gp"
        self.__attached_object_publisher.publish(aco)

        rospy.sleep(0.1)

    def remove_objects(self, names):
        '''
        Removes a list of collision objects from the planning scene.
        :param names: list of str, names of the collision objects
        '''
        for name in names:
            co = moveit_msgs.msg.CollisionObject()
            co.operation = moveit_msgs.msg.CollisionObject.REMOVE
            co.id = name
            self.__collision_object_publisher.publish(co)

            aco = moveit_msgs.msg.AttachedCollisionObject()
            aco.object = co
            aco.link_name = "gp"
            self.__attached_object_publisher.publish(aco)

        rospy.sleep(0.1)