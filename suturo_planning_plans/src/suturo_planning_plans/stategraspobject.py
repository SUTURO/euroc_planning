import smach
import rospy
import utils


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')

        print 'Trying to grasp:\n' + str(userdata.object_to_move.object.id)
        grasp_result = utils.manipulation.grasp(userdata.object_to_move.object)
        print 'Grasp result:' + str(grasp_result)

        if grasp_result:
            return 'success'
        else:
            return 'fail'