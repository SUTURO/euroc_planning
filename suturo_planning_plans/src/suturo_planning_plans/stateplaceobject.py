import smach
import rospy
import utils
from geometry_msgs.msg import PointStamped


class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')

        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        destination.point = userdata.yaml.target_zones[0].target_position
        utils.manipulation.place(destination)

        return 'success'