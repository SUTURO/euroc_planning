import smach
import rospy
from suturo_planning_interface import utils
from suturo_planning_perception import perception
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse


class PoseEstimateObject(smach.State):

    NAME_SERVICE = 'suturo/state/pose_estimate_object'

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_pose_estimate_object)

    def _handle_pose_estimate_object(self, req):
        rospy.loginfo('Executing state PoseEstimateObject')
        taskdata = req.taskdata
        taskdata.focused_object = taskdata.object_to_focus

        taskdata.sec_try = False
        taskdata.fitted_object = None

        # Get the ID of the classified object from the YAML file
        ids = utils.get_yaml_objects_nrs(taskdata.yaml, taskdata.focused_object.object.id)

        # pose_estimated = perception.get_gripper_perception(pose_estimation=True, object_ids=ids)[0] # OLD
        pose_estimated_objects = perception.get_gripper_perception(pose_estimation=True, cuboid=False, object_ids=ids)
        rospy.logdebug("Returned pose_estimated_objects:")
        rospy.logdebug(str(pose_estimated_objects))

        if pose_estimated_objects is None:
            rospy.logwarn('Couldn\'t get gripper perception.')
            if not taskdata.sec_try_done:
                taskdata.sec_try = True
            return TaskDataServiceResponse(taskdata = taskdata, result = "fail")

        if not pose_estimated_objects:
            rospy.logdebug('No object found for pose estimation.')
            if not taskdata.sec_try_done:
                taskdata.sec_try = True
            return TaskDataServiceResponse(taskdata = taskdata, result = "fail")


        # Convert the poses to odom_combined
        map(lambda obj: utils.euroc_object_to_odom_combined(obj), pose_estimated_objects)

        # TODO Investigate the result for the object that's closest to the original object we were interested in
        # TODO find proper threshold
        corresponding_object_idx = utils.get_nearest_object_idx(taskdata.focused_object, pose_estimated_objects, 0.1)

        # # TODO if more than one object were recognized, classify again and take the closest
        # if len(pose_estimated_objects) == 1:
        #     corresponding_object_idx = 0
        # else:
        #     corresponding_object_idx = None
        #     for idx in range(0, len(pose_estimated_objects)):
        #         if pose_estimated_objects[idx].mpe_success:
        #             corresponding_object_idx = idx

        # TODO Call the perception again
        if corresponding_object_idx is None:
            rospy.logdebug('Couldn\'t find the desired object on the next lookup again')
            if not taskdata.sec_try_done:
                taskdata.sec_try = True
            return TaskDataServiceResponse(taskdata = taskdata, result = "fail")


        pose_estimated = pose_estimated_objects[corresponding_object_idx]
        rospy.logdebug('Use Result from Object :%s' % str(pose_estimated))

        if pose_estimated is None or not pose_estimated.mpe_success:
            rospy.logwarn('Poseestimation failed.')
            if not taskdata.sec_try_done:
                taskdata.sec_try = True
            return TaskDataServiceResponse(taskdata = taskdata, result = "fail")


        taskdata.fitted_object = pose_estimated

        rospy.loginfo('Publishing objects into planning scene.')
        utils.publish_collision_objects([pose_estimated.mpe_object])

        return TaskDataServiceResponse(taskdata = taskdata, result = "success")






