__author__ = 'tobi'
import time
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
from suturo_perception_msgs.msg._EurocObject import EurocObject
from suturo_planning_manipulation.calc_grasp_position import get_pre_grasp
from suturo_planning_manipulation.place import get_place_position, get_pre_place_position, get_place_position_for_puzzle
from suturo_planning_manipulation.manipulation_constants import *
from suturo_msgs.msg import Task
from suturo_planning_manipulation.mathemagie import *
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_planning_plans import utils

class PlaceObject(object):

    _retry = 0
    SERVICE_NAME = "sutro/state/place_object"

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)

    def __handle_request(self, request):
        taskdata = request.taskdata
        result = self.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)

    def execute(self, taskdata):
        rospy.loginfo('Executing state PlaceObject')

        destination = taskdata.target_position

        if self._retry == 2:
            rospy.logwarn('Failed to place two times. Dropping object.')
            self._retry = 0
            if not utils.manipulation.open_gripper():
                #cant happen
                taskdata.failed_object = None
                return 'fail'

        if taskdata.enable_movement:
            move_to_func = utils.manipulation.move_arm_and_base_to
        else:
            move_to_func = utils.manipulation.move_to

        co = utils.manipulation.get_planning_scene().get_attached_object()
        if co is None:
            self._retry = 0
            taskdata.failed_object = None
            return 'noObjectAttached'
        co = co.object
        rospy.logdebug("Placing: " + str(co))

        destination = utils.manipulation.transform_to(destination)
        rospy.logdebug("at::: " + str(destination))
        if taskdata.yaml.task_type == Task.TASK_5:
            destination.point.z = 0.4
            dest_orientation = deepcopy(taskdata.yaml.puzzle_fixture.orientation)
            place_poses = get_place_position_for_puzzle(destination, dest_orientation)
        else:
            place_poses = get_place_position(co, destination, utils.manipulation.transform_to, taskdata.dist_to_obj,
                                         taskdata.grasp)
        print place_poses
        # place_poses = utils.map.filter_invalid_poses3(destination.point.x, destination.point.y, place_poses)
        if not taskdata.enable_movement:
            place_poses = utils.manipulation.filter_close_poses(place_poses)

        for place_pose in place_poses:
            rospy.logdebug("Try to place at: " + str(place_pose))

            if taskdata.yaml.task_type != Task.TASK_5:
                if not move_to_func(get_pre_place_position(place_pose), do_not_blow_up_list=(co.id)):
                    rospy.logwarn("Can't reach preplaceposition.")
                    continue
                else:
                    rospy.logdebug("preplaceposition taken")

                time.sleep(0.5)
                rospy.sleep(1)

            if not move_to_func(place_pose, do_not_blow_up_list=(co.id, "map")):
                rospy.logwarn("Can't reach placeposition.")
                continue
            else:
                rospy.logdebug("placeposition taken")

            time.sleep(0.5)
            rospy.sleep(1)

            self._retry = 0
            gripper_target = min(utils.manipulation.get_current_gripper_state()[1] + 0.002, gripper_max_pose)
            if not utils.manipulation.open_gripper(gripper_target):
                #cant happen
                taskdata.failed_object = None
                return 'fail'

            rospy.sleep(3)
            if not utils.manipulation.open_gripper():
                #cant happen
                taskdata.failed_object = None
                return 'fail'

            if taskdata.yaml.task_type == Task.TASK_5:
                utils.manipulation.get_planning_scene().remove_object(co.id)

            time.sleep(0.5)
            rospy.sleep(1)

            # post_place_pose = utils.manipulation.transform_to(place_pose, co.id)
            if not move_to_func(get_pre_grasp(place_pose), do_not_blow_up_list=(co.id, "map")) and \
                    not move_to_func(get_pre_place_position(place_pose), do_not_blow_up_list=(co.id, "map")):
                rospy.logwarn("Can't reach postplaceposition. Continue anyway")
                taskdata.failed_object = None
                self._retry = 0
                return 'success'
            else:
                rospy.logdebug("postplaceposition taken")
            rospy.sleep(0.25)
            rospy.loginfo("placed " + co.id)
            taskdata.failed_object = None
            self._retry = 0
            return 'success'

        #try to place the object where it currently is
        rospy.logdebug("Placement failed, to to place where we are.")
        taskdata.place_position = self.new_place_position()
        o = EurocObject
        o.mpe_object = CollisionObject()
        o.object = CollisionObject()
        o.object.id = co.id
        o.mpe_object.id = co.id
        o.c_centroid = Point(0,0,0)
        taskdata.failed_object = o

        self._retry += 1
        return 'noPlacePosition'

    def new_place_position(self):
        current_pose = utils.manipulation.get_arm_move_group().get_current_pose()
        destination = PointStamped()
        destination.header.frame_id = current_pose.header.frame_id
        destination.point = current_pose.pose.position
        destination.point.z = 0
        return destination