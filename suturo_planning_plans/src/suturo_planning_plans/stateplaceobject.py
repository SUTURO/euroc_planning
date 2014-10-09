from geometry_msgs.msg._PoseStamped import PoseStamped
import smach
import rospy
from geometry_msgs.msg import PointStamped
from suturo_planning_manipulation.calc_grasp_position import get_pre_grasp
from suturo_planning_manipulation.place import get_place_position, get_pre_place_position

import utils


class PlaceObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'noObjectAttached', 'noPlacePosition'],
                             input_keys=['enable_movement', 'target_position', 'grasp'],
                             output_keys=['place_position'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')

        destination = userdata.target_position

        if userdata.enable_movement:
            move_to_func = utils.manipulation.move_arm_and_base_to
        else:
            move_to_func = utils.manipulation.move_to

        co = utils.manipulation.get_planning_scene().get_attached_object()
        if co is None:
            return 'noObjectAttached'
        co = co.object

        destination = utils.manipulation.transform_to(destination)
        place_pose = get_place_position(co, destination, utils.manipulation.tf_listener(), utils.manipulation.transform_to, userdata.grasp)

        if place_pose is None:
            # try to place the object at the current position
            # current_pose = utils.manipulation.get_arm_move_group().get_current_pose()
            # destination = PointStamped()
            # destination.header.frame_id = current_pose.header.frame_id
            # destination.point = current_pose.pose.position
            # destination.point.z = 0
            userdata.place_position = self.new_place_position()

            return 'noPlacePosition'

        if not move_to_func(get_pre_place_position(place_pose)):
            rospy.logwarn("Can't reach preplaceposition.")
            userdata.place_position = self.new_place_position()

            return 'noPlacePosition'
        else:
            rospy.logdebug("preplaceposition taken")
        rospy.sleep(1)
        if not move_to_func(place_pose):
            rospy.logwarn("Can't reach placeposition.")
            userdata.place_position = self.new_place_position()

            return 'noPlacePosition'
        else:
            rospy.logdebug("placeposition taken")

        rospy.sleep(1)
        if not utils.manipulation.open_gripper():
            #cant happen
            return 'fail'
        rospy.sleep(1)

        post_place_pose = utils.manipulation.transform_to(place_pose, co.id)
        # post_place_pose.header.frame_id = "/tcp"
        # post_place_pose.pose.position = Point(0, 0, -post_place_length)

        if not move_to_func(get_pre_grasp(post_place_pose)):
            rospy.logwarn("Can't reach postplaceposition. Continue anyway")
            return 'success'
        else:
            rospy.logdebug("postplaceposition taken")

        rospy.sleep(0.25)
        rospy.loginfo("placed " + co.id)
        return 'success'


        # destination = PointStamped()
        # destination.header.frame_id = '/odom_combined'
        # destination.point = None
        # for target_zone in userdata.yaml.target_zones:
        #     if target_zone.expected_object == userdata.object_to_move.mpe_object.id:
        #         rospy.loginfo('Placing object on location %s' % target_zone.name)
        #         destination.point = target_zone.target_position
        #
        # if destination.point is None:
        #     rospy.logdebug('No target zone found.')
        #     return 'fail'
        # else:
        #     if userdata.enable_movement:
        #         placed = utils.manipulation.place_and_move(destination)
        #     else:
        #         placed = utils.manipulation.place(destination)
        #
        # if placed:
        #     return 'success'
        # else:
        #     return 'fail'

    def new_place_position(self):
        current_pose = utils.manipulation.get_arm_move_group().get_current_pose()
        destination = PointStamped()
        destination.header.frame_id = current_pose.header.frame_id
        destination.point = current_pose.pose.position
        destination.point.z = 0
        return destination