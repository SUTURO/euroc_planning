import rospy
import scipy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


class SearchGrid:

    field = None
    coordinates = None

    def __init__(self, num_of_fields_x, num_of_fields_y, size_x, size_y):
        self.field = scipy.zeros([num_of_fields_x, num_of_fields_y])
        self.coordinates = self._get_real_coordinates(size_x, size_y)

    def _get_real_coordinates(self, size_x, size_y):
        field_size_x = 1.0 * size_x / len(self.field)
        start_x = (-size_x / 2.0) + (field_size_x / 2)

        field_size_y = 1.0 * size_y / len(self.field[0])
        start_y = (-size_y / 2.0) + (field_size_y / 2)

        grid_coordinates = scipy.zeros([len(self.field), len(self.field[0]), 3])
        for x in range(0, len(grid_coordinates)):
            for y in range(0, len(grid_coordinates[0])):
                x_coord = start_x + (x * field_size_x)
                y_coord = start_y + (y * field_size_y)
                grid_coordinates[x][y] = [x_coord, y_coord, 0]

        return grid_coordinates

    def to_marker_array(self):
        markers = []

        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[0])):
                marker = Marker()

                marker.header.stamp = rospy.get_rostime()
                marker.header.frame_id = '/odom_combined'
                marker.ns = 'suturo_planning/search_grid'
                marker.id = x + (y * len(self.field))
                marker.action = Marker.ADD
                marker.type = Marker.CUBE
                marker.pose.position.x = self.coordinates[x, y][0]
                marker.pose.position.y = self.coordinates[x, y][1]
                marker.pose.position.z = self.coordinates[x, y][2]
                marker.pose.orientation.w = 1
                marker.scale.x = 2.0 / len(self.coordinates)
                marker.scale.y = marker.scale.x
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
        return ColorRGBA(1, max(1 - (value * 0.2), 0), max(1 - (value * 0.2), 0), 1)