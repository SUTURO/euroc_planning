import rospy
import scipy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class SearchGrid:

    field = None

    def __init__(self, num_of_fields_x, num_of_fields_y):
        self.field = scipy.zeros([num_of_fields_x, num_of_fields_y])

    def to_marker_array(self):

        markers = []
        distance = 2.0 / len(self.field)
        start_point = -1.0 + (distance / 2.0)
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                marker = Marker()

                marker.header.stamp = rospy.get_rostime()
                marker.header.frame_id = '/odom_combined'
                marker.ns = 'search_grid'
                marker.id = x + (y * len(self.field))
                marker.action = Marker.ADD
                marker.type = Marker.CUBE
                marker.pose.position.x = start_point + (x * distance)
                marker.pose.position.y = start_point + (y * distance)
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = distance
                marker.scale.y = distance
                marker.scale.z = 0.01
                marker.color = SearchGrid._get_color_for_value(self.field[x][y])
                marker.lifetime = rospy.Time(0)

                markers.append(marker)

        return MarkerArray(markers)

    @staticmethod
    def _get_color_for_value(value):
        # If there is an obstacle
        if value < 0:
            return ColorRGBA(0, 0, 0, 1)

        # Color depending on how often it was seen
        return ColorRGBA(255, 255 - (value * 20), 255 - (value * 20), 1)