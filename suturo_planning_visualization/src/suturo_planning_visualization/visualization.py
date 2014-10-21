from geometry_msgs.msg._Point import Point
import rospy
from visualization_msgs.msg import Marker, MarkerArray

pub_marker = None
pub_marker_array = None


def publish_marker(marker):
    global pub_marker
    if pub_marker is None:
        pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
        rospy.sleep(1.0)

    pub_marker.publish(marker)
    rospy.sleep(0.1)


def publish_marker_array(marker_array):
    global pub_marker_array
    if pub_marker_array is None:
        pub_marker_array = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        rospy.sleep(1.0)

    pub_marker_array.publish(marker_array)
    rospy.sleep(0.1)

    #for marker in marker_array.markers:
    #    publish_marker(marker)
    #    rospy.sleep(0.05)


def publish_pose_stamped(pose_stamped):
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
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    marker.lifetime = rospy.Time(0)

    publish_marker(marker)


def publish_vector(vector, name):
    marker = Marker()
    marker.id = name
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.ns = 'suturo_planning/search'
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = '/odom_combined'
    marker.pose.position.x = vector[0]
    marker.pose.position.y = vector[1]
    marker.pose.position.z = vector[2]
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1
    marker.lifetime = rospy.Time(0)

    publish_marker(marker)


def publish_lines(from_point, to_points):
    markers = []

    for i in range(0, len(to_points)):
        markers.append(make_line(from_point, to_points[i], i))

    publish_marker_array(MarkerArray(markers))


def make_line(point1, point2, marker_id):
    marker = Marker()
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.ns = 'suturo_planning/search'
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = '/odom_combined'
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 0
    marker.color.a = 1
    marker.points = [point1, point2]
    marker.lifetime = rospy.Time(0)

    return marker

def visualize_point(p):
    marker = Marker()
    if type(p) is Point:
        marker.pose.position = p
        marker.header.frame_id = "/odom_combined"
    else:
        marker.pose.position = p.point
        marker.header.frame_id = p.header.frame_id
    global pub_marker
    if pub_marker is None:
        pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)
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
    pub_marker.publish(marker)
    rospy.sleep(0.1)

def visualize_poses(poses):
    global pub_marker
    if pub_marker is None:
        pub_marker = rospy.Publisher('visualization_marker', Marker, queue_size=10)
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
        pub_marker.publish(marker)
        rospy.sleep(0.1)
    for i in range(len(poses), 50):
        marker.action = Marker.DELETE
        marker.id = i
        pub_marker.publish(marker)
        rospy.sleep(0.1)
