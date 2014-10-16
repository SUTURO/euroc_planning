import smach
import rospy
import utils
from utils import *
from suturo_planning_perception import perception
from suturo_perception_msgs.msg import EurocObject


class ClassifyObjects(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'objectsClassified'],
                             input_keys=['object_to_perceive', 'yaml', 'placed_objects', 'objects_found'],
                             output_keys=['classified_objects'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        rospy.loginfo('Using simple perception.')
        get_gripper = perception.get_gripper_perception()
        if get_gripper is None:
            rospy.logwarn("get_gripper is None!!!?!?")
            get_gripper = []
        perceived_objects = get_valid_objects(get_gripper)
        rospy.logdebug('Found ' + str(len(get_gripper)) + ' objects, ' + str(len(perceived_objects)) + ' are valid.')
        if not perceived_objects:
            return 'noObject'

        collision_objects = []
        matched_objects = []
        found_object_names = map(lambda obj: obj.mpe_object.id, userdata.objects_found)

        for obj in perceived_objects:
            # classify object
            matched_obj = classify_object(obj)
            rospy.logdebug('Matched: \n' + str(obj) + '\n----------------------------------\nwith: ' + str(matched_obj))

            if matched_obj.c_type == EurocObject.OBSTACLE:
                if matched_obj.c_cuboid_success:
                    collision_objects.append(matched_obj.object)
                continue

            if matched_obj.c_type == EurocObject.UNKNOWN or matched_obj.c_type == EurocObject.TABLE:
                if matched_obj.c_cuboid_success:
                    collision_objects.append(matched_obj.object)
                # TODO something better than ignoring the problem
                rospy.logwarn('Unknown object perceived or table. Ignoring it for now.')
                continue

            if matched_obj.c_type == EurocObject.OBJECT:
                # check if the object was already placed
                if matched_obj.object.id in found_object_names:
                    rospy.logdebug('Object %s was already seen.' % matched_obj.object.id)
                else:
                    matched_objects.append(matched_obj)

        rospy.loginfo('Publishing objects into planning scene.')
        # publish_collision_objects(collision_objects)
        userdata.classified_objects = matched_objects

        # check if it was an object
        if matched_objects:
            return 'objectsClassified'
        else:
            return 'noObject'