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
                             input_keys=['object_to_perceive', 'yaml', 'placed_objects'],
                             output_keys=['object_to_move', 'pending_objects', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        perceived_objects = get_valid_objects(perception.get_gripper_perception())
        if not perceived_objects:
            userdata.objects_found = []
            return 'noObject'

        collision_objects = []
        matched_objects = []

        for obj in perceived_objects:
            # classify object
            matched_obj = classify_object(obj)
            rospy.logdebug('Matched: ' + str(obj) + '\n----------------------------------\nwith: ' + str(matched_obj))

            # check if the object was already placed
            if not matched_obj.object.id in userdata.placed_objects:
                matched_objects.append(matched_obj)
                collision_objects.append(matched_obj.object)

        publish_collision_objects(collision_objects)
        userdata.objects_found = matched_objects

        # check if it was an object
        object_candidate = get_object_to_move(matched_objects)
        if object_candidate is not None:
            userdata.object_to_move = object_candidate
            return 'validObject'
        else:
            return 'noObject'