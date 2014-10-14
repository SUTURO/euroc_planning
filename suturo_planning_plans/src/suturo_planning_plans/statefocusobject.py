import smach
import rospy
from suturo_planning_plans import utils


class FocusObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['focusedObject', 'fail'],
                             input_keys=['yaml', 'object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state FocusObject')

        poses = []  # TODO get the pose from manipulation with object_to_move
        for pose in poses:
            if utils.manipulation.move_to(pose):
                return 'focusedObject'

        return 'fail'