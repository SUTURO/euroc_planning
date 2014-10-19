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
        smach.State.__init__(self, outcomes=['mapScanned'],
                             input_keys=[],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanMapMastCam')

        # print utils.map
        # arm_base = utils.manipulation.get_base_origin()

            # print len(next_point)

        next_point = utils.map.get_closest_unknown()
        while not len(next_point) == 0:
            map_updated = False
            for i in xrange(len(next_point)):
                print next_point[i]
                next_point[i].z = 0.08
                poses = make_scan_pose(next_point[i], 0.55, 0.65, n=16)
                # visualize_poses(poses)
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
                        continue
                    utils.map.publish_as_marker()
                    rospy.logdebug("published")
                    co = utils.map.to_collision_object()
                    utils.manipulation.get_planning_scene().add_object(co)
                    break
            if not map_updated:
                rospy.loginfo("can't update map any further")
                break
            next_point = utils.map.get_closest_unknown()
            print len(next_point)

        return 'mapScanned'




    # (next_point, centroid) = utils.map.get_next_point(arm_base.point.x, arm_base.point.y)
    #     while len(next_point) != 0:
    #         if len(next_point) == 1:
    #             print next_point[0]
    #             next_point[0].z = 0.075
    #             poses = make_scan_pose(next_point[0], 0.35, pi/5, n=16)
    #             # visualize_poses(poses)
    #             j = 0
    #             move_successfull = False
    #             while j < len(poses) and not move_successfull:
    #                 move_successfull = utils.manipulation.move_arm_and_base_to(poses[j])
    #                 j += 1
    #             if move_successfull:
    #                 rospy.sleep(3)
    #                 arm_base = utils.manipulation.get_base_origin()
    #                 utils.map.add_point_cloud(arm_base.point, scene_cam=False)
    #                 utils.map.publish_as_marker()
    #                 rospy.logdebug("published")
    #                 # utils.manipulation.get_planning_scene().remove_object("map")
    #                 co = utils.map.to_collision_object()
    #                 utils.manipulation.get_planning_scene().add_object(co)
    #
    #                 (next_point, centroid) = utils.map.get_next_point(arm_base.point.x, arm_base.point.y)
    #             # print len(next_point)
    #             continue
    #
    #         for i in xrange(len(next_point)):
    #             print next_point[i]
    #             print centroid
    #             next_point[i].z = 0.075
    #             # poses = make_scan_pose(next_point[i], 0.35, pi/5, n=16)
    #             poses = PoseStamped()
    #             poses.header.frame_id = "/odom_combined"
    #             poses.pose.position = deepcopy(next_point[i])
    #             poses.pose.position.z = 0.075
    #             poses.pose.position = subtract_point(centroid, poses.pose.position)
    #             poses.pose.position = set_vector_length(magnitude(poses.pose.position)+0.35, poses.pose.position)
    #             poses.pose.position = add_point(centroid, poses.pose.position)
    #             poses.pose.orientation = three_points_to_quaternion(poses.pose.orientation, centroid)
    #             print poses
    #
    #             poses = [poses]
    #             # visualize_poses(poses)
    #             j = 0
    #             move_successfull = False
    #             while j < len(poses) and not move_successfull:
    #                 move_successfull = utils.manipulation.move_arm_and_base_to(poses[j])
    #                 j += 1
    #             if move_successfull:
    #                 rospy.sleep(2)
    #                 arm_base = utils.manipulation.get_base_origin()
    #                 utils.map.add_point_cloud(arm_base.point, scene_cam=False)
    #                 utils.map.publish_as_marker()
    #                 rospy.logdebug("published")
    #                 # utils.manipulation.get_planning_scene().remove_object("map")
    #                 co = utils.map.to_collision_object()
    #                 utils.manipulation.get_planning_scene().add_object(co)
    #
    #                 (next_point, centroid) = utils.map.get_next_point(arm_base.point.x, arm_base.point.y)