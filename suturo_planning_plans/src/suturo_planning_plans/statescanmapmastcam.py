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
        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        map = Map(2,2)
        arm_base = utils.manipulation.get_base_origin()
        print arm_base
        utils.manipulation.set_cam_pan(0.275)
        utils.manipulation.set_cam_tilt(0.775)
        rospy.sleep(3)
        map.add_point_cloud(arm_origin=arm_base, scene_cam=True)

        utils.manipulation.set_cam_pan(-0.275)
        rospy.sleep(3)
        map.add_point_cloud(arm_origin=arm_base, scene_cam=True)

        utils.manipulation.set_cam_pan(0)
        utils.manipulation.set_cam_tilt(1.1)
        rospy.sleep(3)
        map.add_point_cloud(arm_origin=arm_base, scene_cam=True)

        map.publish_as_marker()

        userdata.map = map
        cos = map.get_collision_objects()
        utils.manipulation.get_planning_scene().add_objects(cos)
        return 'mapScanned'

