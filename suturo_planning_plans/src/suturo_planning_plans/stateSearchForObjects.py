__author__ = 'ichumuh'
from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.mathemagie import *
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose, make_grasp_pose
from suturo_planning_search.map import Map
from suturo_planning_visualization.visualization import visualize_poses

import utils
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import pi


class SearchForObjects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')

        # print utils.map
        # arm_base = utils.manipulation.get_base_origin()

            # print len(next_point)


        return 'mapScanned'
