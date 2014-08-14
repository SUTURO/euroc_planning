import rospy
from visualization_msgs.msg import Marker

pub = None


def visualize(pose_stamped):
    global pub
    if pub is None:
        rospy.publisher('visualization_marker', Marker, queue_size=10)

    marker = Marker()
    marker.id = pose_stamped.header.seq
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.ns = 'suturo_planning'
    marker.pose = pose_stamped.pose
    marker.header = pose_stamped.header
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.1
    marker.color.r = 255
    marker.color.g = 255
    marker.color.b = 0
    marker.color.a = 1