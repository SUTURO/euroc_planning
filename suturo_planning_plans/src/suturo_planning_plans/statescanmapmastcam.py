from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.mathemagie import set_vector_length, add_point
from suturo_planning_search.map import Map

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import *
from suturo_msgs.msg import Task
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class ScanMapMastCam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=['enable_movement', 'yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')
        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        utils.map = Map(2)
        # arm_base = utils.manipulation.get_base_origin()
        # rospy.sleep(4)

        print("task_type = "+str(userdata.yaml.task_type))
        if userdata.yaml.task_type == Task.TASK_5:
            rospy.logdebug("adding puzzle fixture to planning scene...")
            fixture = CollisionObject()
            fixture_box = SolidPrimitive()
            fixture_box.type = SolidPrimitive.BOX
            fixture_box.dimensions.append(None)
            fixture_box.dimensions.append(None)
            fixture_box.dimensions.append(None)
            fixture_box.dimensions[SolidPrimitive.BOX_X] = 0.25
            fixture_box.dimensions[SolidPrimitive.BOX_Y] = 0.25
            fixture_box.dimensions[SolidPrimitive.BOX_Z] = 0.1
            fixture_pose = userdata.yaml.puzzle_fixture
            fixture.primitives.append(fixture_box)
            fixture.primitive_poses.append(fixture_pose)
            fixture.id = "fixture"
            utils.manipulation.get_planning_scene().add_object(fixture)

        utils.manipulation.pan_tilt(0.2, 0.5)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(0.2825, 0.775)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(0, 1.1)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(-0.2825, 0.775)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)


        utils.manipulation.pan_tilt(-0.2, 0.5)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)
        utils.manipulation.pan_tilt(0, 0.45)

        if not userdata.enable_movement:
            return 'mapScanned'

        co = utils.map.to_collision_object()
        utils.manipulation.get_planning_scene().add_object(co)
        # utils.manipulation.get_planning_scene().add_object(co)
        rospy.sleep(2)
        n = 8
        for i in range(0, n):
            a = 2 * pi * ((i + 0.0) / (n + 0.0))

            goal_base_pose = PoseStamped()
            goal_base_pose.header.frame_id = "/odom_combined"

            goal_base_pose.pose.position= Point(cos(a), sin(a), 0)
            if goal_base_pose.pose.position.x < 0 and goal_base_pose.pose.position.y < 0 or \
                                    goal_base_pose.pose.position.x > 0 and goal_base_pose.pose.position.y > 0:
                continue

            goal_base_pose.pose.position = set_vector_length(0.25, goal_base_pose.pose.position)

            goal_base_pose.pose.orientation.w = 1

            if utils.manipulation.move_base(goal_base_pose):
                rospy.sleep(utils.waiting_time_before_scan)
                utils.map.add_point_cloud(arm_origin=utils.manipulation.get_base_origin().point, scene_cam=True)
                break

        # utils.map.add_point_cloud(arm_origin=arm_base, scene_cam=True)
        # utils.map.add_point_cloud(arm_origin=arm_base, scene_cam=True)

        # utils.map.publish_as_marker()

        co = utils.map.to_collision_object()
        utils.manipulation.get_planning_scene().add_object(co)
        return 'mapScanned'

