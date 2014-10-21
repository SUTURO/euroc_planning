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


class ScanMapArmCam(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned', 'newImage'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')

        # arm_base = utils.manipulation.get_base_origin()
        eef_pose = utils.manipulation.get_eef_position()
        next_point = utils.map.get_closest_unknown(eef_pose.pose.position)
        map_updated = False
        # last_point_id = 0
        for i in xrange(len(next_point)):
            cell_x = next_point[i].x
            cell_y = next_point[i].y
            utils.map.mark_cell(cell_x, cell_y, True)
            next_point[i].z = 0.08
            poses = make_scan_pose(next_point[i], 0.55, 0.65, n=16)
            for (c, x, y) in utils.map.get_surrounding_cells8(cell_x, cell_y):
                if not c.is_free():
                    p = Point(x, y, 0)
                    p2 = Point(cell_x, cell_y, 0)
                    cell_to_p = subtract_point(p, p2)
                    poses = filter(lambda pose: get_angle(cell_to_p, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), p2)) > pi/4, poses)
            eef_pose = utils.manipulation.get_eef_position()
            poses.sort(key=lambda pose: euclidean_distance(pose.pose.position, eef_pose.pose.position))
            visualize_poses(poses)
            j = 0
            move_successfull = False
            while j < len(poses) and not move_successfull:
                move_successfull = utils.manipulation.move_arm_and_base_to(poses[j])
                j += 1
            if move_successfull:
                rospy.sleep(4)
                arm_base = utils.manipulation.get_base_origin()
                map_updated = utils.map.add_point_cloud(arm_base.point, scene_cam=False)
                if not map_updated:
                    rospy.logdebug("no map update")
                    utils.map.mark_cell(next_point[i].x, next_point[i].y, False)
                    continue
                rospy.logdebug("published")
                co = utils.map.to_collision_object()
                utils.manipulation.get_planning_scene().add_object(co)
                utils.map.mark_cell(next_point[i].x, next_point[i].y, False)
                # last_point_id = i
                return 'newImage'
            utils.map.mark_cell(next_point[i].x, next_point[i].y, False)
        if not map_updated:
            rospy.loginfo("can't update map any further")
            utils.map.all_unknowns_to_obstacle()
        else:
            rospy.logerr("IT HAPPEND!!!!")
            # return 'mapScanned'
        # next_point = utils.map.get_closest_unknown(arm_base.point)
        # next_point = utils.map.get_closest_unknown(next_point[last_point_id])
        # print len(next_point)

        return 'mapScanned'


