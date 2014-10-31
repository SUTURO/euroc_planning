import rospy
import smach
from suturo_planning_plans import utils


class ScanShadow(smach.State):

    # _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ScanShadow')
        wait = 10

        scan_poses = ['shadow_pose1', 'shadow_pose2']

        for scan_pose in scan_poses:

            rospy.loginfo('Taking pose: %s' % scan_pose)
            utils.manipulation.move_to(scan_pose)
            rospy.sleep(wait)

            # Add pc to map
            rospy.logdebug('Adding current point cloud to map')
            arm_base = utils.manipulation.get_base_origin()
            utils.map.add_point_cloud(arm_base.point, scene_cam=False)

        # Creating planning scene from map
        rospy.logdebug('Creating planning scene from map')
        utils.map.all_unknowns_to_obstacle()
        rospy.logdebug(str(utils.map))
        co = utils.map.to_collision_object()
        utils.manipulation.get_planning_scene().add_object(co)

        return 'success'