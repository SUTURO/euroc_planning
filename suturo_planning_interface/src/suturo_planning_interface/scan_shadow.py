__author__ = 'andreas'

import rospy
import smach
from suturo_planning_interface import utils
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse


class ScanShadow(smach.State):

    # _ctr = 0

    NAME_SERVICE = 'suturo/state/scan_shadow'
    RET_VAL_DONE = 'success'

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_scan_shadow)

    def _handle_scan_shadow(self, data):
        rospy.loginfo('Executing state ScanShadow')
        scan_poses = ['shadow_pose1', 'shadow_pose2']

        for scan_pose in scan_poses:
            rospy.loginfo('Taking pose: %s' % scan_pose)
            utils.manipulation.move_to(scan_pose)
            rospy.sleep(utils.waiting_time_before_scan)

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

        return TaskDataServiceResponse(taskdata = data.taskdata, result = self.RET_VAL_DONE)
