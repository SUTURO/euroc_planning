import smach
import rospy
import utils
from utils import *
from suturo_planning_perception import perception


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive', 'yaml', 'placed_objects'],
                             output_keys=['object_to_move', 'pending_objects', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        rospy.loginfo('Using simple perception.')
        get_gripper = perception.get_gripper_perception()
        perceived_objects = get_valid_objects(get_gripper)
        rospy.logdebug('Found ' + str(len(get_gripper)) + ' objects, ' + str(len(perceived_objects)) + ' are valid.')
        if not perceived_objects:
            userdata.objects_found = []
            return 'noObject'

        collision_objects = []
        matched_objects = []

        for obj in perceived_objects:
            # classify object
            matched_obj = classify_object(obj)
            rospy.logdebug('Matched: ' + str(obj) + '\n----------------------------------\nwith: ' + str(matched_obj))

            if matched_obj is None:
                continue

            # check if the object was already placed
            if not matched_obj.object.id in userdata.placed_objects:
                rospy.loginfo('Using pose estimation.')

                pose_estimated = get_pose_estimation(userdata, matched_obj)

                if (pose_estimated is None) or (not pose_estimated.mpe_success):
                    pose_estimated = get_pose_estimation(userdata, matched_obj)

                if pose_estimated is None:
                    continue

                if pose_estimated.mpe_success:
                    pose_estimated.object.id = pose_estimated.mpe_object.id
                    matched_objects.append(pose_estimated)
                    collision_objects.append(pose_estimated.mpe_object)

        publish_collision_objects(collision_objects)
        userdata.objects_found = matched_objects

        # check if it was an object
        object_candidate = get_object_to_move(matched_objects)
        if object_candidate is not None:
            userdata.object_to_move = object_candidate
            return 'validObject'
        else:
            return 'noObject'


def get_pose_estimation(userdata, obj):
    # Get the ID of the classified object from the YAML file
    ids = get_yaml_objects_nrs(userdata.yaml, obj.object.id)

    # pose_estimated = perception.get_gripper_perception(pose_estimation=True, object_ids=ids)[0] # OLD
    pose_estimated_objects = perception.get_gripper_perception(pose_estimation=True, object_ids=ids)

    # TODO Investigate the result for the object that's closest to the original object we were interested in
    # TODO find proper threshold
    corresponding_object_idx = get_nearest_object_idx(obj, pose_estimated_objects, 0.1)
    # TODO Call the perception again
    if corresponding_object_idx is None:
        rospy.logdebug('Couldnt find the desired object on the next lookup again')
        return None

    pose_estimated = pose_estimated_objects[corresponding_object_idx]
    rospy.logdebug('Use Result from Object :%s' % str(pose_estimated))

    return pose_estimated