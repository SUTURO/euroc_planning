import smach
import rospy
import utils
from utils import *
from suturo_planning_perception import perception
from suturo_perception_msgs.msg import EurocObject


class PoseEstimateObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['focused_object', 'yaml'],
                             output_keys=['fitted_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PoseEstimateObject')

        # Get the ID of the classified object from the YAML file
        ids = get_yaml_objects_nrs(userdata.yaml, userdata.focused_object.object.id)

        # pose_estimated = perception.get_gripper_perception(pose_estimation=True, object_ids=ids)[0] # OLD
        pose_estimated_objects = perception.get_gripper_perception(pose_estimation=True, cuboid=False, object_ids=ids)

        if pose_estimated_objects is None:
            rospy.logwarn('Couldn\'t get gripper perception.')
            return 'fail'

        if not pose_estimated_objects:
            rospy.logdebug('No object found for pose estimation.')
            return 'fail'

        # TODO Investigate the result for the object that's closest to the original object we were interested in
        # TODO find proper threshold
        # corresponding_object_idx = get_nearest_object_idx(userdata.focused_object, pose_estimated_objects, 0.1)
        corresponding_object_idx = 0
        # TODO Call the perception again
        if corresponding_object_idx is None:
            rospy.logdebug('Couldn\'t find the desired object on the next lookup again')
            return 'fail'

        pose_estimated = pose_estimated_objects[corresponding_object_idx]
        rospy.logdebug('Use Result from Object :%s' % str(pose_estimated))

        if pose_estimated is None:
            rospy.logwarn('Poseestimation failed.')
            return 'fail'

        userdata.fitted_object = pose_estimated
        publish_collision_objects([pose_estimated.mpe_object])

        return 'success'