import rospy
import scipy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

__author__ = 'ichumuh'


class Map:

    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2

    def __init__(self, num_of_fields_x, num_of_fields_y, size_x, size_y):
        self.field = scipy.zeros([num_of_fields_x, num_of_fields_y])
        self.cell_size_x = 1.0 * size_x / len(self.field)
        self.cell_size_y = 1.0 * size_y / len(self.field[0])
        self.size_x = size_x
        self.size_y = size_y
        # self.coordinates = self._get_real_coordinates(size_x, size_y)

    def __del__(self):
        pass

    def add_point_cloud(self, point_cloud):
        pass

    def get_cell(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x, y)
        # print x_index
        # print y_index
        return self.field[x_index][y_index]

    def get_cell_by_index(self, x, y):
        return self.field[x][y]

    def set_cell(self, x, y, value):
        (x_index, y_index) = self.coordinates_to_index(x, y)
        self.set_cell_by_index(x_index, y_index, value)

    def set_cell_by_index(self, x, y, value):
        self.field[x][y] = value

    def coordinates_to_index(self, x, y):
        x_index = int((x + (self.size_x/2)) / self.cell_size_x)
        y_index = int((y + (self.size_y/2)) / self.cell_size_y)
        if x_index == 100:
            x_index = 99

        if y_index == 100:
            y_index = 99
        return (x_index, y_index)

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
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = 2.0 / len(self.coordinates)
                marker.scale.y = marker.scale.x
                marker.scale.z = 0.01
                marker.color = SearchGrid._get_color_for_value(self.field[x][y])
                marker.lifetime = rospy.Time(0)

                markers.append(marker)

        return MarkerArray(markers)