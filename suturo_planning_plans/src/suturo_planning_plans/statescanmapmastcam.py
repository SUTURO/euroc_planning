import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_search.map import Map

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import pi

class ScanMapMastCam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')
        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        utils.map = Map(2)
        arm_base = utils.manipulation.get_base_origin()
        print arm_base
        utils.manipulation.pan_tilt(0.2, 0.5)
        rospy.sleep(4)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(0.275, 0.775)
        rospy.sleep(4)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(0, 1.1)
        rospy.sleep(4)
        utils.map.add_point_cloud(scene_cam=True)

        utils.manipulation.pan_tilt(-0.275, 0.775)
        rospy.sleep(4)
        utils.map.add_point_cloud(scene_cam=True)


        utils.manipulation.pan_tilt(-0.2, 0.5)
        rospy.sleep(4)
        # utils.manipulation.pan_tilt(0, 0.6)
        # rospy.sleep(5)
        utils.map.add_point_cloud(scene_cam=True)

        # utils.map.add_point_cloud(arm_origin=arm_base, scene_cam=True)
        # utils.map.add_point_cloud(arm_origin=arm_base, scene_cam=True)

        # utils.map.publish_as_marker()

        co = utils.map.to_collision_object()
        utils.manipulation.get_planning_scene().add_object(co)
        return 'mapScanned'

