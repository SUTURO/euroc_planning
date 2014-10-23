from OpenSSL import rand
from random import randint
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

    finished = False
    last_scanned_point = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned', 'newImage'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')

        if self.finished:
            return 'mapScanned'

        regions = utils.map.get_unknown_regions()

        last_point = self.last_scanned_point
        if last_point is None:
            eef_pose = utils.manipulation.get_eef_position()
            last_point = eef_pose.pose.position

        #evt nach groesse sortieren?
        # regions.sort(key=lambda r: euclidean_distance( Point(*(utils.map.index_to_coordinates(*r.get_avg())+(0,))), last_point))
        regions.sort(key=lambda r: -r.get_number_of_cells())


        i = 0
        r_id = 0
        boarder_cells = []
        for i in range(len(regions)):
            boarder_cells.append(utils.map.get_boarder_cells_points(regions[i]))
            #evt mit dem punkt anfangen der am weitesten von der mitte weg ist?
            boarder_cells[-1].sort(key=lambda cp: euclidean_distance(cp, last_point))

        while reduce(lambda l, bc: l + len(bc), boarder_cells, 0) > 0 and utils.map.get_percent_cleared() < 0.9:
            if len(boarder_cells[r_id]) == 0:
                r_id += 1
                r_id = r_id % len(regions)
                i = 0
                continue

            next_point = boarder_cells[r_id][0]
            cell_x = next_point.x
            cell_y = next_point.y
            utils.map.mark_cell(cell_x, cell_y, True)
            next_point.z = 0.085
            poses = make_scan_pose(next_point, 0.5, 0.7, n=16)
            for (c, x, y) in utils.map.get_surrounding_cells8(cell_x, cell_y):
                if not c.is_free():
                    p = Point(x, y, 0)
                    p2 = Point(cell_x, cell_y, 0)
                    cell_to_p = subtract_point(p, p2)
                    poses = filter(lambda pose: get_angle(cell_to_p, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), p2)) > pi/4, poses)
            # eef_pose = utils.manipulation.get_eef_position()
            poses.sort(key=lambda pose: -euclidean_distance( Point(*(utils.map.index_to_coordinates(*regions[r_id].get_avg())+(0,))), Point(pose.pose.position.x, pose.pose.position.y, 0)))
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
                    utils.map.mark_cell(next_point.x, next_point.y, False)
                    continue
                rospy.logdebug("published")
                co = utils.map.to_collision_object()
                utils.manipulation.get_planning_scene().add_object(co)
                utils.map.mark_cell(next_point.x, next_point.y, False)
                self.last_scanned_point = next_point
                return 'newImage'
            utils.map.mark_cell(next_point.x, next_point.y, False)
            boarder_cells[r_id].remove(next_point)
            i += 1
            if i >= 1:

                r_id += 1
                r_id = r_id % len(regions)
                i = 0

        rospy.loginfo("can't update map any further")
        utils.map.all_unknowns_to_obstacle()
        self.finished = True
        return 'mapScanned'


