#!/usr/bin/env python
from moveit_msgs.msg._AttachedCollisionObject import AttachedCollisionObject
from suturo_msgs.msg._Task import Task

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
from copy import deepcopy


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
        """
        Adds a big flat box to the planning scene.
        :param height: z value
        :type: float
        """
        pose = PoseStamped()
        pose.header.frame_id = "/odom_combined"
        pose.pose.position = Point(0, 0, height)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        box = self.make_box("ground" + str(height), pose, [3, 3, 0.01])
        self.safe_objects.append(box.id)
        self.add_object(box)

    def add_yaml(self, yaml):
        """
        Adds some fixed collision objects that are defined in the yaml description
        :param yaml: yaml description of the task
        :type: Task
        """
        self.add_ground()
        if yaml is None:
            self.add_cam_mast(0.92, 0.92)
        else:
            x = yaml.mast_of_cam.base_pose.linear.x
            y = yaml.mast_of_cam.base_pose.linear.y
            self.add_cam_mast(x, y)
            if len(yaml.relative_puzzle_part_target_poses) == 0:
                return
            pose = PoseStamped()
            pose.header.frame_id = "/odom_combined"
            pose.pose = deepcopy(yaml.puzzle_fixture)
            #TODO: WTF?
            #if you change the Point shit here, also change it in map.py!
            # pose.pose.position = add_point(pose.pose.position, Point(0.115 if yaml.puzzle_fixture.position.x < 0 else -0.18, 0.165 if yaml.puzzle_fixture.position.y < 0 else 0.05, 0))
            pose.pose.position = get_puzzle_fixture_center(yaml.puzzle_fixture)
            box = self.make_box("puzzle", pose, [0.32, 0.32, 0.1])
            print("puzzle_fixture collision object = "+str(box))
            self.safe_objects.append(box.id)
            self.add_object(box)

    def add_cam_mast(self, x, y):
        """
        Adds a Cylinder where the cam mast is.
        :param x: x coordinate
        :type: float
        :param y: y coordinate
        :type: float
        """
        pose = PoseStamped()
        pose.header.frame_id = "/odom_combined"
        pose.pose.position = Point(x, y, 0.55)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        c1 = self.make_cylinder("cm1", pose, [1.1, 0.05])
        self.safe_objects.append(c1.id)
        self.add_object(c1)

        pose.pose.position = Point(x, y, 1.0)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)
        c1 = self.make_box("mast_cams", pose, [0.3, 0.3, 0.3])
        self.safe_objects.append(c1.id)
        self.add_object(c1)

    @staticmethod
    def make_box(name, pose, size):
        """
        Creates a box collision object.
        :param name: name of the box
        :type: str
        :param pose: position of the box
        :type: PoseStamped
        :param size: box size
        :type: [float(x), float(y), float(z)]
        :return: box collision object
        :type: CollisionObject
        """
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
        """
        Creates a cylinder collision object.
        :param name: name of the cylinder
        :type: str
        :param pose: position of the cylinder
        :type: PoseStamped
        :param size: cylinder size
        :type: [float(height), float(radius)]
        :return: cylinder collisionobject
        :type: CollisionObject
        """
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
        """
        :return: that are currently in the planning scene
        :type: [CollisionObject]
        """
        return self.get_planning_scene().world.collision_objects

    def get_collision_object(self, name):
        """
        Returns a collision object with the given name out of the planning scene.
        :param name: name of the desired object
        :type: str
        :return: desired collision object or None
        :type: CollisionObject or None
        """
        cos = self.get_collision_objects()
        for co in cos:
            if co.id == name:
                return co
        return None

    def get_attached_objects(self):
        """
        :return: list of attached CollisionObject
        :type: [AttachedCollisionObject]
        """
        return self.get_planning_scene().robot_state.attached_collision_objects

    def get_attached_object(self):
        """
        :return: the attached CollisionObject
        :type: AttachedCollisionObject or None
        """
        acos = self.get_attached_objects()
        if len(acos) == 0:
            rospy.logerr("No objects attached!")
            return None
        else:
            return acos[0]

    def get_planning_scene(self):
        """
        :return: the PlanningScene
        """
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