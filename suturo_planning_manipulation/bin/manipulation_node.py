#!/usr/bin/env python
import rospy
from threading import Thread
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Empty
from suturo_manipulation_msgs.srv import *
from suturo_planning_manipulation import manipulation_constants
from suturo_planning_manipulation.manipulation import Manipulation
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.manipulation_constants import MOVE_SERVICE, MOVE_WITH_PLAN_SERVICE, \
    ADD_COLLISION_OBJECTS_SERVICE, GET_COLLSISION_OBJECT_SERVICE, MOVE_MASTCAM_SERVICE, OPEN_GRIPPER_SERVICE, \
    CLOSE_GRIPPER_SERVICE, BASE_ORIGIN_TOPIC, GET_EEF_POSITION_TOPIC, CREATE_POSES_FOR_OBJECT_SCANNING_SERVICE, \
    BLOW_DOWN_OBJECTS_SERVICE, BLOW_UP_OBJECTS_SERVICE
from suturo_planning_manipulation.manipulation_constants import PLAN_SERVICE
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_interface import utils
from suturo_planning_visualization.visualization import visualize_poses

__author__ = 'hansa'


class ManipulationNode(object):

    def __init__(self):
        print("init ManipulationNode")
        rospy.init_node("Manipulation_Control")
        self.__manipulation = Manipulation()
        rospy.Service(MOVE_SERVICE, Move, self.__handle_move)
        rospy.Service(PLAN_SERVICE, Plan, self.__handle_plan)
        rospy.Service(MOVE_WITH_PLAN_SERVICE, MoveWithPlan, self.__handle_move_with_plan)
        rospy.Service(ADD_COLLISION_OBJECTS_SERVICE, AddCollisionObjects, self.__handle_add_objects)
        rospy.Service(GET_COLLSISION_OBJECT_SERVICE, GetCollisionObject, self.__handle_get_collision_object)
        rospy.Service(MOVE_MASTCAM_SERVICE, MoveMastCam, self.__handle_mast_cam)
        rospy.Service(OPEN_GRIPPER_SERVICE, OpenGripper, self.__handle_open_gripper)
        rospy.Service(CLOSE_GRIPPER_SERVICE, CloseGripper, self.__handle_close_gripper)
        rospy.Service(CREATE_POSES_FOR_OBJECT_SCANNING_SERVICE, CreatePosesForScanning,
                      self.__handle_create_poses_for_scanning)
        rospy.Service(BLOW_UP_OBJECTS_SERVICE, BlowUpObjects, self.__handle_blow_up_objects)
        rospy.Service(BLOW_DOWN_OBJECTS_SERVICE, Empty, self.__handle_blow_down_objects)

        self.__base_publisher = rospy.Publisher(BASE_ORIGIN_TOPIC, PointStamped)
        self.__eef_position_publisher = rospy.Publisher(GET_EEF_POSITION_TOPIC, PoseStamped)

    def __handle_create_poses_for_scanning(self, msg):
        __region_centroid = msg.centroid
        __angle = msg.angle
        __distance = msg.distance
        __quantity = msg.quantity
        poses = make_scan_pose(__region_centroid, __distance, __angle, n=__quantity)
        poses = self.__manipulation.filter_close_poses(poses)
        rospy.loginfo("utils.map id ="+ str(hex(id(utils.map))))

        rospy.wait_for_service('/suturo/environment/filter_invalid_scan_poses2')
        try:
            filter_invalid_scan_poses2 = rospy.ServiceProxy('/suturo/environment/filter_invalid_scan_poses2', FilterPoses)
            print "Handle Create Poses For Scanning: try filter"
            rospy.loginfo("Handle Create Poses For Scanning: try filter")
            resp = filter_invalid_scan_poses2(__region_centroid.x, __region_centroid.y, poses)
            poses = resp.poses
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
        visualize_poses(poses)
        return CreatePosesForScanningResponse(poses)

    def __handle_blow_down_objects(self, msg):
        self.__manipulation.blow_down_objects()

    def __handle_blow_up_objects(self, msg):
        self.__manipulation.blow_up_objects(msg.do_not_blow_up_list, msg.distance)
    
    def __handle_move(self, msg):
        goal_pose = self.__get_goal_pose(msg)

	print("########################################################################################################################################################################################")
	print(msg)
	print(goal_pose)
	print("########################################################################################################################################################################################")

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
        return AddCollisionObjectsResponse()

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
        if msg.position != 0:
            position = msg.position
        rospy.loginfo("Openening gripper to pose: %d" % position)
	for i in range(10):
            rospy.loginfo(i)
	return self.__manipulation.open_gripper(position)

    def __handle_close_gripper(self, msg):
        obj = msg.object
        if obj == CollisionObject():
            obj = None
        grasp_point = msg.grasp_point
        if grasp_point == PointStamped():
            grasp_point = None
        result = self.__manipulation.close_gripper(obj, grasp_point)
        joint_state = self.__manipulation.get_current_gripper_state()
        return (result, joint_state)

    def __publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.__base_publisher.publish(self.__manipulation.get_base_origin())
            self.__eef_position_publisher.publish(self.__manipulation.get_eef_position())
            rate.sleep()

    def start(self):
        publisher_thread = Thread(target=self.__publish)
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
    print("manipulation_node name == main")
    node = ManipulationNode()
    node.start()
