import smach
import rospy
from geometry_msgs.msg import PointStamped

import utils


class PlaceObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'object_to_move', 'enable_movement'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')

        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        destination.point = None
        for target_zone in userdata.yaml.target_zones:
            if target_zone.expected_object == userdata.object_to_move.mpe_object.id:
                rospy.loginfo('Placing object on location %s' % target_zone.name)
                destination.point = target_zone.target_position

        if destination.point is None:
            rospy.logdebug('No target zone found.')
            return 'fail'
        else:
            if userdata.enable_movement:
                placed = utils.manipulation.place_and_move(destination)
            else:
                placed = utils.manipulation.place(destination)

        if placed:
            return 'success'
        else:
            return 'fail'