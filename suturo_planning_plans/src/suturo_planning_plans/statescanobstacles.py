from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_search.map import Map

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import pi

class ScanObstacles(smach.State):

    obstacle_cluster = []
    next_cluster = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanObstacles')
        if len(self.obstacle_cluster) == 0:
            self.obstacle_cluster = utils.map.get_obstacle_regions()
            self.obstacle_cluster.sort(key=lambda x: x.get_number_of_cells())

        if self.next_cluster >= len(self.obstacle_cluster):
            rospy.loginfo("searched all cluster")
            return 'finish'

        #such obstacle region
        current_region = self.obstacle_cluster[self.next_cluster]
        p = Point(*tuple(current_region.get_avg)+(0,))



        #hinfahren
        #gucken


        return 'mapScanned'

