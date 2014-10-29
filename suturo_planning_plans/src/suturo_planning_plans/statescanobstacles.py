from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
import time
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_search.map import Map
from suturo_planning_visualization.visualization import visualize_poses

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import pi

class ScanObstacles(smach.State):

    obstacle_cluster = []
    next_cluster = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned', 'noRegionLeft', 'newImage'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanObstacles')
        if len(self.obstacle_cluster) == 0:
            self.obstacle_cluster = utils.map.get_obstacle_regions()
            rospy.logdebug(str(len(self.obstacle_cluster)) + " regions found.")
            self.obstacle_cluster.sort(key=lambda x: x.get_number_of_cells())

        if self.next_cluster >= len(self.obstacle_cluster):
            rospy.loginfo("searched all cluster")
            return 'noRegionLeft'

        current_region = self.obstacle_cluster[self.next_cluster]
        self.next_cluster += 1

        rospy.logdebug("current region: " + str(self.next_cluster) + "\n" + str(current_region))

        region_centroid = Point(*(utils.map.index_to_coordinates(*current_region.get_avg()))+(-0.065,))

        poses = make_scan_pose(region_centroid, 0.75, 0.9, n=16)
        visualize_poses(poses)
        for pose in poses:
            if utils.manipulation.move_arm_and_base_to(pose):

                rospy.logdebug('Wait for clock')
                time.sleep(3)
                #
                # rospy.logdebug('Wait for tf again.')
                # rospy.sleep(4)
                return 'newImage'

        return 'mapScanned'

