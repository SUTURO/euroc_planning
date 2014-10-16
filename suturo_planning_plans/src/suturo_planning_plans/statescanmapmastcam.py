import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_search.map import Map

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception


class ScanMapMastCam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=['map'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')
        map = Map(2,2)
        utils.manipulation.set_cam_pan(0.275)
        utils.manipulation.set_cam_tilt(0.775)
        rospy.sleep(4)
        map.add_point_cloud(scene_cam=True)

        utils.manipulation.set_cam_pan(-0.275)
        rospy.sleep(4)
        map.add_point_cloud(scene_cam=True)

        utils.manipulation.set_cam_pan(0)
        utils.manipulation.set_cam_tilt(1.1)
        rospy.sleep(4)
        map.add_point_cloud(scene_cam=True)

        map.publish_as_marker()

        userdata.map = map
        return 'mapScanned'

