#!/usr/bin/env python
from threading import Thread
from geometry_msgs.msg import PointStamped
import moveit_commander
import moveit_msgs
import rospy
from suturo_planning_manipulation.srv import *
from suturo_planning_manipulation.manipulation import Manipulation


__author__ = 'hansa'


class ManipulationNode(object):

    _MOVE_GROUP_MAPPING = {
        PlanRequest.MOVE_GROUP_ARM: moveit_commander.MoveGroupCommander("arm"),
        PlanRequest.MOVE_GROUP_ARM_BASE: moveit_commander.MoveGroupCommander("arm_base"),
        PlanRequest.MOVE_GROUP_BASE: moveit_commander.MoveGroupCommander("base"),
        PlanRequest.MOVE_GROUP_GRIPPER: moveit_commander.MoveGroupCommander("gripper")
    }

    def __init__(self):
        rospy.init_node("Manipulation_Control")
        self.__manipulation = Manipulation()
        rospy.Service('suturo_move', Move, self.__handle_move)
        rospy.Service("suturo_plan", Plan, self.__handle_plan)
        rospy.Service("suturo_move_with_plan", MoveWithPlan, self.__handle_move_with_plan)
        rospy.Service("suturo_add_objects_to_planning_scene", AddPlanningObjects, self.__handle_add_objects)
        self.__publisher = rospy.Publisher("suturo_manipulation_get_base_origin", PointStamped)

    def __handle_move(self, msg):
        goal_pose = self.__get_goal_pose(msg)

        if msg.type == MoveRequest.ACTION_MOVE_TO:
            result = self.__manipulation.move_to(goal_pose, msg.do_not_blow_up_list)
        elif msg.type == MoveRequest.ACTION_MOVE_ARM_AND_BASE_TO:
            result = self.__manipulation.move_arm_and_base_to(goal_pose, msg.do_not_blow_up_list)
        elif msg.type == MoveRequest.ACTION_MOVE_BASE:
            result = self.__manipulation.move_base(goal_pose)
        else:
            result = False
        return MoveResponse(result)

    def __handle_plan(self, msg):
        move_group = self._MOVE_GROUP_MAPPING[msg.move_group]
        plan = self.__manipulation.plan(move_group, msg.goal, msg.start_state, msg.max_movement_time)
        if plan is not None:
            return PlanResponse(plan=plan.motion_plan_response)
        else:
            return PlanResponse()

    def __handle_move_with_plan(self, msg):
        plan = moveit_msgs.srv.GetMotionPlan(motion_plan_response=msg.plan)
        result = self.__manipulation.move_with_plan_to(plan)
        return MoveWithPlanResponse(result=result)

    def __handle_add_objects(self, msg):
        self.__manipulation.get_planning_scene().add_objects(msg.objects)

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
            goal_pose = msg.goal_pose_name
        else:
            goal_pose = msg.goal_pose


if __name__ == "__main__":
    node = ManipulationNode()
    node.start()