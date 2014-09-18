import smach
import rospy
from utils import get_valid_objects
from utils import classify_object
from utils import publish_collision_objects
from utils import get_object_to_move
from suturo_planning_perception import perception


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive', 'yaml'],
                             output_keys=['object_to_move', 'pending_objects', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        perceived_objects = get_valid_objects(perception.get_gripper_perception())
        if not perceived_objects:
            userdata.objects_found = []
            return 'noObject'

        collision_objects = []
        for obj in perceived_objects:
            # obj.object.id = str(obj.c_centroid.x)
            matched_obj = classify_object(obj)
            rospy.logdebug('Matched: ' + str(obj) + '\nwith: ' + str(matched_obj))
            collision_objects.append(obj.object)
            # obj.color = userdata.object_to_perceive.color
        publish_collision_objects(collision_objects)
        userdata.objects_found = perceived_objects

        # check if it was an object
        object_candidate = get_object_to_move(perceived_objects)
        if object_candidate is not None:
            userdata.object_to_move = object_candidate
            return 'validObject'
        else:
            return 'noObject'