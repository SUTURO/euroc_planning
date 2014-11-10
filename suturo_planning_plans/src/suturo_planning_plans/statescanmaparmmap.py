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

        regions.sort(key=lambda r: -r.get_number_of_cells())

        for r in regions:
            r.set_boarder_cells(sorted(utils.map.get_boarder_cell_points(r),
                                       cmp=lambda p1, p2: utils.map.is_more_edge(p1.x, p1.y, p2.x, p2.y)))

        i = 0
        r_id = 0

        while len(regions) > 0 and utils.map.get_percent_cleared() < 0.985:
            if len(regions[r_id].get_boarder_cells()) < self.min_num_of_cells:
                regions.remove(regions[r_id])
                if len(regions) == 0:
                    break
                r_id = r_id % len(regions)
                i = 0
                continue

            if i >= 2:
                r_id += 1
                r_id = r_id % len(regions)
                i = 0
                continue

            next_point = regions[r_id].get_boarder_cells()[-1]

            regions[r_id].get_boarder_cells().remove(next_point)

            cell_x = next_point.x
            cell_y = next_point.y
            utils.map.mark_cell(cell_x, cell_y, True)
            next_point.z = 0.2
            poses = make_scan_pose(next_point, 0.7, 0.8, n=16)
            poses = utils.map.filter_invalid_scan_poses(cell_x, cell_y, poses)
            poses = utils.map.filter_invalid_scan_poses2(cell_x, cell_y, poses)

            poses.sort(key=lambda pose: -euclidean_distance(
                Point(*(utils.map.index_to_coordinates(*regions[r_id].get_avg()) + (0,))),
                Point(pose.pose.position.x, pose.pose.position.y, 0)))
            visualize_poses(poses)
            j = 0
            move_successfull = False
            while j < len(poses) and not move_successfull:
                rospy.logdebug("try scan pose nr. " + str(j))
                move_successfull = utils.manipulation.move_arm_and_base_to(poses[j], blow_up=("all"))
                j += 1
            if move_successfull:
                rospy.sleep(utils.waiting_time_before_scan)
                arm_base = utils.manipulation.get_base_origin()
                map_updated = utils.map.add_point_cloud(arm_base.point, scene_cam=False)
                if not map_updated:
                    rospy.logdebug("no map update")
                    utils.map.mark_cell(next_point.x, next_point.y, False)
                    i += 1
                    continue
                rospy.logdebug("published")
                co = utils.map.to_collision_object()
                utils.manipulation.get_planning_scene().add_object(co)
                utils.map.mark_cell(next_point.x, next_point.y, False)
                return 'newImage'
            utils.map.mark_cell(next_point.x, next_point.y, False)

            i += 1

        rospy.loginfo("can't update map any further")
        utils.map.all_unknowns_to_obstacle()
        self.finished = True
        return 'mapScanned'


