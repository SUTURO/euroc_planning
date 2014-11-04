from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
import time
from suturo_planning_manipulation import mathemagie
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_manipulation.mathemagie import subtract_point, get_angle, euclidean_distance
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
                             input_keys=['enable_movement', 'sec_try'],
                             output_keys=['sec_try_done', 'focused_point'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanObstacles')
        userdata.sec_try_done = False
        if userdata.sec_try:
            current_region = self.obstacle_cluster[self.next_cluster-1]
            userdata.sec_try_done = True
        else:
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

        dist_to_region = mathemagie.euclidean_distance(Point(0, 0, 0), region_centroid)

        # If the arm cannot move ignore distant regions
        # TODO find the best max distance
        if not userdata.enable_movement:
            rospy.logdebug('Distance of the current region to the arm: %s' % str(dist_to_region))
            if dist_to_region > 1.1:
                rospy.logwarn('Current region is out of reach. Ignoring it.')
                return 'mapScanned'

        # angle = (pi / 2) - (dist_to_region / 1)
        angle = 1.2
        distance = 0.6 + current_region.get_number_of_cells()*0.01

        rospy.logdebug('Focusing point: %s' % str(region_centroid))
        rospy.logdebug('Angle: %s' % str(angle))
        rospy.logdebug('Distance: %s' % str(distance))
        poses = make_scan_pose(region_centroid, distance, angle, n=16)

        if not userdata.enable_movement:
            # c_to_base = subtract_point(Point(0, 0, 0), region_centroid)
            # poses = [pose for pose in poses if get_angle(subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), region_centroid), c_to_base) > pi/4]
            poses = utils.manipulation.filter_close_poses(poses)

        poses = utils.map.filter_invalid_poses2(region_centroid.x, region_centroid.y, poses)

        if userdata.sec_try:
            current_pose = utils.manipulation.get_eef_position().pose.position
            current_pose.z = 0
            region_to_eef = subtract_point(region_centroid, current_pose)

            poses.sort(key=lambda pose: abs(get_angle(region_to_eef, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), region_centroid)) - pi/2))

        visualize_poses(poses)

        if userdata.enable_movement:
            move = utils.manipulation.move_arm_and_base_to
        else:
            move = utils.manipulation.move_to

        for pose in poses:
            utils.manipulation.set_planning_time_arm(2)
            if move(pose):
                utils.manipulation.set_planning_time_arm(5)
                userdata.focused_point = region_centroid

                rospy.logdebug('Wait for clock')
                time.sleep(3)
                #
                # rospy.logdebug('Wait for tf again.')
                # rospy.sleep(4)
                return 'newImage'

        return 'mapScanned'
