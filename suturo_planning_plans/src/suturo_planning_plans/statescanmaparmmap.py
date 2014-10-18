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

        next_point = utils.map.get_closest_unknown()
        while not len(next_point) == 0:
            for i in range(0, len(next_point)):
                print next_point[i]
                next_point[i].z = 0.075
                poses = make_scan_pose(next_point[i], 0.35, pi/5, n=16)
                # visualize_poses(poses)
                j = 0
                move_successfull = False
                while j < len(poses) and not move_successfull:
                    move_successfull = utils.manipulation.move_arm_and_base_to(poses[j])
                    j += 1
                if move_successfull:
                    break

            rospy.sleep(3)
            arm_base = utils.manipulation.get_base_origin()
            utils.map.add_point_cloud(arm_base.point, scene_cam=False)
            utils.map.publish_as_marker()
            rospy.logdebug("published")
            utils.manipulation.get_planning_scene().remove_object("map")
            co = utils.map.to_collision_object()
            utils.manipulation.get_planning_scene().add_object(co)

            next_point = utils.map.get_closest_unknown()
            print len(next_point)

        return 'mapScanned'