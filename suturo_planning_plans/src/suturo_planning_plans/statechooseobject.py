import smach
import rospy
from geometry_msgs.msg import PointStamped


class ChooseObject(smach.State):

    _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectChosen', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found', 'placement_failed', 'placed_objects'],
                             output_keys=['object_to_move'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')

        #set the place destination
        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        destination.point = None
        for target_zone in userdata.yaml.target_zones:
            if target_zone.expected_object == userdata.object_to_move.mpe_object.id:
                rospy.loginfo('Placing object on location %s' % target_zone.name)
                destination.point = target_zone.target_position
                userdata.place_position = destination

        if len(userdata.objects_found) > self._ctr:
            userdata.object_to_move = userdata.objects_found[self._ctr]
            self._ctr += 1
            return 'objectChosen'
        else:
            self._ctr = 0
            return 'noObjectsLeft'