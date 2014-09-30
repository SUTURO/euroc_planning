import rospy
from visualization_msgs.msg import Marker, MarkerArray

pub_marker = None
pub_marker_array = None


def publish_marker(marker):
    global pub_marker
    if pub_marker is None:
        pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

    pub_marker.publish(marker)


def publish_marker_array(marker_array):
    global pub_marker_array
    if pub_marker_array is None:
        pub_marker_array = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)

    pub_marker_array.publish(marker_array)


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
    marker.color.r = 255
    marker.color.g = 255
    marker.color.b = 0
    marker.color.a = 1
    marker.lifetime = rospy.Time(0)

    publish_marker(marker)


def publish_vector(vector, name):
    marker = Marker()
    marker.id = name
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.ns = 'search_grid'
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
    marker.color.b = 255
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
    marker.ns = 'suturo_planning'
    marker.header.stamp = rospy.get_rostime()
    marker.header.frame_id = '/odom_combined'
    marker.scale.x = 0.02
    marker.scale.y = 0.02
    marker.scale.z = 0.02
    marker.color.r = 255
    marker.color.g = 255
    marker.color.b = 0
    marker.color.a = 1
    marker.points = [point1, point2]
    marker.lifetime = rospy.Time(0)

    return marker