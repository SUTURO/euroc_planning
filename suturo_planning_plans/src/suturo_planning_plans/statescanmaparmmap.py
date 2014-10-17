import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_search.map import Map
from suturo_planning_visualization.visualization import visualize_poses

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import pi


class ScanMapArmCam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')

        # print utils.map
        arm_base = utils.manipulation.get_base_origin()

        next_point = utils.map.get_closest_unknown()
        # while not next_point is None:
            # print next_point
        poses = make_scan_pose(next_point, 0.5, pi/6, n=16)
        visualize_poses(poses)
        i = 0
        while i<len(poses) and not utils.manipulation.move_arm_and_base_to(poses[i]):
            i += 1
        rospy.sleep(3)
        utils.map.add_point_cloud(arm_base, scene_cam=False)
        utils.map.publish_as_marker()
        rospy.logdebug("published")
        utils.manipulation.get_planning_scene().remove_object("map")
        co = utils.map.to_collision_object()
        utils.manipulation.get_planning_scene().add_object(co)

            # next_point = utils.map.get_closest_unknown()

        return 'mapScanned'