from geometry_msgs.msg._Point import Point
import rospy
from visualization_msgs.msg import Marker

pub = None


def visualize(pose_stamped):
    global pub
    if pub is None:
        rospy.Publisher('visualization_marker', Marker, queue_size=10)

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

def visualize_poses(poses):
    global pub
    if pub is None:
        pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.sleep(0.5)
    # r = rospy.Rate(1)  # 10hz

    marker = Marker()
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "mani"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.scale.z = 0.02
    marker.scale.y = 0.04
    marker.scale.x = 0.06

    i = 0
    for grasp in poses:
        marker.header.frame_id = grasp.header.frame_id
        marker.id = i
        marker.pose = grasp.pose
        i += 1
        pub.publish(marker)
        rospy.sleep(0.25)

def visualize_point(p):
    marker = Marker()
    if type(p) is Point:
        marker.pose.position = p
        marker.header.frame_id = "/odom_combined"
    else:
        marker.pose.position = p.point
        marker.header.frame_id = p.header.frame_id
    global pub
    if pub is None:
        pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    # pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        rospy.sleep(0.5)


    marker.header.stamp = rospy.get_rostime()
    marker.ns = "mani"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    marker.scale.z = 0.03
    marker.scale.y = 0.03
    marker.scale.x = 0.03

    marker.id = 500
    marker.pose.orientation.w = 1
    pub.publish(marker)
    rospy.sleep(0.1)