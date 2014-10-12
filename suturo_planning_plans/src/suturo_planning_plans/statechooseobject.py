import smach
import rospy
from geometry_msgs.msg import PointStamped


class ChooseObject(smach.State):

    _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectChosen', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['object_to_move', 'place_position'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')

        if len(userdata.objects_found) > self._ctr:
            chosen_object = userdata.objects_found[self._ctr]
            userdata.object_to_move = chosen_object
            self._ctr += 1
        else:
            self._ctr = 0
            return 'noObjectsLeft'

        #set the place destination
        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        for target_zone in userdata.yaml.target_zones:
            if target_zone.expected_object == chosen_object.mpe_object.id:
                rospy.loginfo('Placing object on location %s' % target_zone.name)
                destination.point = target_zone.target_position
                userdata.place_position = destination
                break

        return 'objectChosen'