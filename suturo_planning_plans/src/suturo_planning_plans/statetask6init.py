import smach
import rospy
from suturo_msgs.msg import Task
import utils

class Task6Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state CamToDropzone')

        # TODO: 1. Foerderband in PS haun, wenns noch nicht drin ist.
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.addConveyorBelt)

        return 'fail'

    def addConveyorBelt(self, msg):
        conveyorBeltMsg = msg.conveyorBelt
        ps = utils.manipulation.get_planning_scene()
        pass