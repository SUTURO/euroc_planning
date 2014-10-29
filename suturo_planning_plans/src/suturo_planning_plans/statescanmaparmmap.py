from random import randint
import smach
import rospy
from suturo_planning_manipulation.mathemagie import *
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose, make_grasp_pose
from suturo_planning_visualization.visualization import visualize_poses

import utils
from math import pi


class ScanMapArmCam(smach.State):

    finished = False
    last_scanned_point = None
    min_num_of_cells = 2

    def __init__(self):
        smach.State.__init__(self, outcomes=['mapScanned', 'newImage'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapArmCam')

        if self.finished:
            return 'mapScanned'

        regions = utils.map.get_unknown_regions()

        last_point = self.last_scanned_point
        if last_point is None:
            eef_pose = utils.manipulation.get_eef_position()
            last_point = eef_pose.pose.position

        #evt nach groesse sortieren?
        # regions.sort(key=lambda r: euclidean_distance( Point(*(utils.map.index_to_coordinates(*r.get_avg())+(0,))), last_point))
        # regions = filter(lambda r: r.get_number_of_cells > self.min_num_of_cells, regions)
        regions.sort(key=lambda r: -r.get_number_of_cells())

        for r in regions:
            r.set_boarder_cells(sorted(utils.map.get_boarder_cells_points(r), cmp=lambda p1, p2: utils.map.is_more_edge(p1.x, p1.y, p2.x, p2.y)))

        i = 0
        r_id = 0

        while len(regions) > 0 and utils.map.get_percent_cleared() < 0.99:
            if len(regions[r_id].get_boarder_cells()) < self.min_num_of_cells:
                regions.remove(regions[r_id])
                r_id = r_id % len(regions)
                continue

            if  i >= 2:
                r_id += 1
                r_id = r_id % len(regions)
                i = 0
                continue

            next_point = regions[r_id].get_boarder_cells()[-1]

            regions[r_id].get_boarder_cells().remove(next_point)

            cell_x = next_point.x
            cell_y = next_point.y
            utils.map.mark_cell(cell_x, cell_y, True)
            next_point.z = 0.1
            poses = make_scan_pose(next_point, 0.9, 0.8, n=16)
            for (c, x, y) in utils.map.get_surrounding_cells8(cell_x, cell_y):
                if not c.is_free():
                    p = Point(x, y, 0)
                    p2 = Point(cell_x, cell_y, 0)
                    cell_to_p = subtract_point(p, p2)
                    for pose in poses:
                        #filter poses that are pointing to unknowns or obstacles
                        if get_angle(cell_to_p, subtract_point(Point(pose.pose.position.x, pose.pose.position.y, 0), p2)) < pi/4:
                            poses.remove(pose)
                        #filter poses that are outside of the table
                        elif not (-1.25 <= pose.pose.position.x <= 1.25 and -1.25 <= pose.pose.position.y <= 1.25):
                            poses.remove(pose)
                        #filter poses that are above unknowns
                        elif -1.0 <= pose.pose.position.x <= 1.0 and -1.0 <= pose.pose.position.y <= 1.0 :
                            cell = utils.map.get_cell(pose.pose.position.x, pose.pose.position.y)
                            if cell.is_unknown():
                                poses.remove(pose)
                            elif cell.is_obstacle() and cell.highest_z >= pose.pose.position.z - 0.05:
                                poses.remove(pose)

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

            i += 1

        rospy.loginfo("can't update map any further")
        utils.map.all_unknowns_to_obstacle()
        self.finished = True
        rospy.logdebug("Map after arm Scan: " + str(utils.map))
        return 'mapScanned'


