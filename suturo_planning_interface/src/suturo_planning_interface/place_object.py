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
from suturo_planning_interface import utils
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import suturo_interface_msgs.srv

class PlaceObject(object):

    SERVICE_NAME = "suturo/manipulation/place_object"

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, suturo_interface_msgs.srv.PlaceObject, self.__handle_request)

    def __handle_request(self, req):
        result = self.place_object(req.place_position, req.grasp_position)
        return suturo_interface_msgs.srv.PlaceObjectResponse(result)

    def execute(self, destination, grasp_position):
        rospy.loginfo('Executing PlaceObject')

        move_to_func = utils.manipulation.move_to

        co = utils.manipulation.get_planning_scene().get_attached_object()
        co = co.object
        rospy.logdebug("Placing: " + str(co))

        destination = utils.manipulation.transform_to(destination)
        rospy.logdebug("at::: " + str(destination))
        dist_to_obj = self.calculate_distance(grasp_position)
        place_poses = get_place_position(co, destination, utils.manipulation.transform_to, dist_to_obj,
                                         grasp_position)
        print "PlacePoses = "+str(place_poses)
        # place_poses = utils.map.filter_invalid_poses3(destination.point.x, destination.point.y, place_poses)

        ## Only do this when the base cannot move
        place_poses = utils.manipulation.filter_close_poses(place_poses)

        for place_pose in place_poses:
            rospy.logdebug("Try to place at: " + str(place_pose))


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

            gripper_target = min(utils.manipulation.get_current_gripper_state()[1] + 0.002, gripper_max_pose)
            if not utils.manipulation.open_gripper(gripper_target):
                #cant happen
                return False

            rospy.sleep(3)
            if not utils.manipulation.open_gripper():
                #cant happen
                return False

            time.sleep(0.5)
            rospy.sleep(1)

            # post_place_pose = utils.manipulation.transform_to(place_pose, co.id)
            if not move_to_func(get_pre_grasp(place_pose), do_not_blow_up_list=(co.id, "map")) and \
                    not move_to_func(get_pre_place_position(place_pose), do_not_blow_up_list=(co.id, "map")):
                rospy.logwarn("Can't reach postplaceposition. Continue anyway")
                return True
            else:
                rospy.logdebug("postplaceposition taken")
            rospy.sleep(0.25)
            rospy.loginfo("placed " + co.id)
            return True

        return False

    def new_place_position(self):
        current_pose = utils.manipulation.get_arm_move_group().get_current_pose()
        destination = PointStamped()
        destination.header.frame_id = current_pose.header.frame_id
        destination.point = current_pose.pose.position
        destination.point.z = 0
        return destination

    def calculate_distance(self, grasp_position):
        fingertip = get_fingertip(grasp_position)
        fingertip_to_tcp = subtract_point(grasp_position.pose.position, fingertip.point)
        dist_to_obj = magnitude(fingertip_to_tcp)
        return dist_to_obj