import smach
import rospy
import utils
from suturo_planning_perception import perception


class PoseEstimateObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['focused_object', 'yaml'],
                             output_keys=['fitted_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PoseEstimateObject')

        userdata.fitted_object = None

        # Get the ID of the classified object from the YAML file
        ids = utils.get_yaml_objects_nrs(userdata.yaml, userdata.focused_object.object.id)

        # pose_estimated = perception.get_gripper_perception(pose_estimation=True, object_ids=ids)[0] # OLD
        pose_estimated_objects = perception.get_gripper_perception(pose_estimation=True, cuboid=False, object_ids=ids)
        rospy.logdebug("Returned pose_estimated_objects:")
        rospy.logdebug(str(pose_estimated_objects))

        if pose_estimated_objects is None:
            rospy.logwarn('Couldn\'t get gripper perception.')
            return 'fail'

        if not pose_estimated_objects:
            rospy.logdebug('No object found for pose estimation.')
            return 'fail'

        # Convert the poses to odom_combined
        map(lambda obj: utils.euroc_object_to_odom_combined(obj), pose_estimated_objects)

        # TODO Investigate the result for the object that's closest to the original object we were interested in
        # TODO find proper threshold
        corresponding_object_idx = utils.get_nearest_object_idx(userdata.focused_object, pose_estimated_objects, 0.1)

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
            return 'fail'

        pose_estimated = pose_estimated_objects[corresponding_object_idx]
        rospy.logdebug('Use Result from Object :%s' % str(pose_estimated))

        if pose_estimated is None or not pose_estimated.mpe_success:
            rospy.logwarn('Poseestimation failed.')
            return 'fail'

        userdata.fitted_object = pose_estimated

        rospy.loginfo('Publishing objects into planning scene.')
        utils.publish_collision_objects([pose_estimated.mpe_object])

        return 'success'