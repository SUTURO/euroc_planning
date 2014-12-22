#!/usr/bin/env python
import rospy
from threading import Thread
from geometry_msgs.msg import PointStamped
from suturo_planning_manipulation.srv import *
from suturo_planning_manipulation import manipulation_constants
from suturo_planning_manipulation.manipulation import Manipulation
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped


__author__ = 'hansa'


class ManipulationNode(object):

    def __init__(self):
        rospy.init_node("Manipulation_Control")
        self.__manipulation = Manipulation()
        rospy.Service("/suturo/manipulation/move", Move, self.__handle_move)
        rospy.Service("/suturo/manipulation/plan", Plan, self.__handle_plan)
        rospy.Service("/suturo/manipulation/move_with_plan", MoveWithPlan, self.__handle_move_with_plan)
        rospy.Service("/suturo/manipulation/add_collision_objects", AddCollisionObjects, self.__handle_add_objects)
        rospy.Service("/suturo/manipulation/get_collision_object", GetCollisionObject, self.__handle_get_collision_object)
        rospy.Service("/suturo/manipulation/move_mastcam", MoveMastCam, self.__handle_mast_cam)
        rospy.Service("/suturo/manipulation/open_gripper", OpenGripper, self.__handle_open_gripper)
        rospy.Service("/suturo/manipulation/close_gripper", CloseGripper, self.__handle_close_gripper)

        self.__publisher = rospy.Publisher("suturo_manipulation_get_base_origin", PointStamped)


    def __handle_move(self, msg):
        goal_pose = self.__get_goal_pose(msg)

        if msg.type == MoveRequest.ACTION_MOVE_ARM_TO:
            result = self.__manipulation.move_to(goal_pose, msg.do_not_blow_up_list)
        elif msg.type == MoveRequest.ACTION_MOVE_ARM_AND_BASE_TO:
            result = self.__manipulation.move_arm_and_base_to(goal_pose, msg.do_not_blow_up_list)
        elif msg.type == MoveRequest.ACTION_MOVE_BASE:
            result = self.__manipulation.move_base(goal_pose)
        else:
            result = False
        return MoveResponse(result)

    def __handle_plan(self, msg):
        func = None
        if msg.move_group == PlanRequest.MOVE_GROUP_ARM:
            func = self.__manipulation.plan_arm_to
        elif msg.move_group == PlanRequest.MOVE_GROUP_ARM_BASE:
            func = self.__manipulation.plan_arm_and_base_to
        goal = self.__get_goal_pose(msg)
        start_state = None
        if msg.has_start_state:
            start_state = msg.start_state
        plan = func(goal, start_state)
        return PlanResponse(plan)

    def __handle_move_with_plan(self, msg):
        result = self.__manipulation.move_with_plan_to(msg.plan)
        return MoveWithPlanResponse(result=result)

    def __handle_add_objects(self, msg):
        self.__manipulation.get_planning_scene().add_objects(msg.objects)

    def __handle_mast_cam(self, msg):
        message = self.__manipulation.pan_tilt(msg.pan, msg.tilt)
        result = False
        if message.find("path finished") != -1:
            result = True
        return MoveMastCamResponse(result)

    def __handle_get_collision_object(self, msg):
        return self.__manipulation.get_planning_scene().get_collision_object(msg.name)

    def __handle_open_gripper(self, msg):
        position = manipulation_constants.gripper_max_pose
        if msg.position is not None:
            position = msg.position
        return self.__manipulation.open_gripper(position)

    def __handle_close_gripper(self, msg):
        obj = msg.object
        if obj == CollisionObject():
            obj = None
        grasp_point = msg.grasp_point
        if grasp_point == PointStamped():
            grasp_point = None
        return self.__manipulation.close_gripper(obj, grasp_point)

    def __publish_origin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__publisher.publish(self.__manipulation.get_base_origin())
            rate.sleep()

    def start(self):
        publisher_thread = Thread(target=self.__publish_origin)
        publisher_thread.start()
        while not rospy.is_shutdown():
            try:
                rospy.spin()
            except KeyboardInterrupt:
                break

    def __get_goal_pose(self, msg):
        if msg.goal_pose_name != '':
            return msg.goal_pose_name
        else:
            return msg.goal_pose


if __name__ == "__main__":
    node = ManipulationNode()
    node.start()