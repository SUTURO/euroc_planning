import smach
import rospy

from suturo_perception_msgs.srv import GetGripper


class FastGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectGrasped', 'timeExpired', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state FastGrasp')
        # TODO: Nach ?? Sekunden time expired werfen
        # create service
        service = rospy.ServiceProxy("/suturo/GetGripper", GetGripper)
        # get the first perception
        resp = service("firstConveyorCall,centroid,cuboid")

        return 'fail'