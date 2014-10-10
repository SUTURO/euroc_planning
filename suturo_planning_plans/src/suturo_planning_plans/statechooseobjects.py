import smach
import rospy


class ChooseObject(smach.State):

    _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectChosen', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found', 'placement_failed', 'placed_objects'],
                             output_keys=['object_to_move'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')

        if len(userdata.objects_found) > self._ctr:
            userdata.object_to_move = userdata.objects_found[self._ctr]
            self._ctr += 1
            return 'objectChosen'
        else:
            self._ctr = 0
            return 'noObjectsLeft'