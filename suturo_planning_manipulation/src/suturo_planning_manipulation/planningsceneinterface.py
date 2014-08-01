import moveit_commander
import moveit_msgs

__author__ = 'ichumuh'

#!/usr/bin/env python
import moveit_msgs.msg
import moveit_msgs.srv
from calc_grasp_position import *


class PlanningSceneInterface(object):
    def __init__(self):
        self.__collision_object_publisher = rospy.Publisher('/collision_object', moveit_msgs.msg.CollisionObject,
                                                            queue_size=10)
        self.__attached_object_publisher = rospy.Publisher('/attached_collision_object',
                                                           moveit_msgs.msg.AttachedCollisionObject,
                                                           queue_size=10)
        rospy.wait_for_service("get_planning_scene")
        self.__ps_service_client = rospy.ServiceProxy('get_planning_scene', moveit_msgs.srv.GetPlanningScene)

        rospy.sleep(1)

    def __del__(self):
        pass

    def get_collision_objects(self):
        return self.get_planning_scene().world.collision_objects

    def get_collision_object(self, name):
        cos = self.get_collision_objects()
        for co in cos:
            if co.id == name:
                return co
        return None

    def get_attached_objects(self):
        return self.get_planning_scene().robot_state.attached_collision_objects

    def get_planning_scene(self):
        try:
            ps = self.__ps_service_client(moveit_msgs.msg.PlanningSceneComponents(1023))
            return ps.scene

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def add_object(self, collision_object):
        collision_object.operation = moveit_msgs.msg.CollisionObject.ADD
        self.__collision_object_publisher.publish(collision_object)
        rospy.sleep(0.1)

    def remove_object(self, name):
        co = moveit_msgs.msg.CollisionObject()
        co.operation = moveit_msgs.msg.CollisionObject.REMOVE
        co.id = name
        self.__collision_object_publisher.publish(co)

        aco = moveit_msgs.msg.AttachedCollisionObject()
        aco.object = co
        aco.link_name = "gp"
        self.__attached_object_publisher.publish(aco)

        rospy.sleep(0.1)