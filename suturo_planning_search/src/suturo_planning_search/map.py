from copy import deepcopy
import struct
import numpy
import rospy
import scipy
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, _get_struct_fmt, read_points
from suturo_perception_msgs.srv._GetPointArray import GetPointArray, GetPointArrayRequest
from visualization_msgs.msg import Marker, MarkerArray
from suturo_planning_visualization import visualization
from transformer import Transformer
# from suturo_perception_msgs.msg import GetPointArray
# import pcl

__author__ = 'ichumuh'

fmt_full = ''


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
        self.tf = Transformer()
        self.tcp_cloud = None

        # rospy.Subscriber("/suturo/euroc_scene_cloud", PointCloud2, self.tcp_callback)
        # self.coordinates = self._get_real_coordinates(size_x, size_y)
        self.__get_point_array = rospy.ServiceProxy('muh', GetPointArray)
        rospy.sleep(1.0)

    def __del__(self):
        pass

    def tcp_callback(self, data):
        self.tcp_cloud = data

    def add_point_cloud(self, scene_cam=True):
        request = GetPointArrayRequest()
        if scene_cam:
            request.pointCloudName = GetPointArrayRequest.SCENE
        else:
            request.pointCloudName = GetPointArrayRequest.TCP

        resp = self.__get_point_array(request)
        points = resp.pointArray
        for i in range(0, len(points)/3, step=3):
            x = points[i]
            y = points[i+1]
            z = points[i+2]
            if 0 <= z <= 0.01:
                self.set_cell(x,y, self.FREE)
            else:
                self.set_cell(x,y, self.OBSTACLE)


    def get_cell(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x, y)
        # print x_index
        # print y_index
        return self.get_cell_by_index(x_index, y_index)

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

    def index_to_coordinates(self, x_index, y_index):
        x = x_index * self.cell_size_x - (self.size_x / 2) + (self.cell_size_x/2)
        y = y_index * self.cell_size_y - (self.size_y / 2) + (self.cell_size_y/2)
        # print x_index, " to ", x
        # print y_index, " to ", y
        # print ""
        return (x, y)

    def publish_as_marker(self):
        visualization.publish_marker_array(self.to_marker_array())

    def to_marker_array(self):
        markers = []

        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[0])):
                marker = Marker()

                marker.header.stamp = rospy.get_rostime()
                marker.header.frame_id = '/odom_combined'
                marker.ns = 'suturo_planning/map'
                marker.id = x + (y * len(self.field))
                marker.action = Marker.ADD
                marker.type = Marker.CUBE
                (x_i, y_i) = self.index_to_coordinates(x, y)
                marker.pose.position.x = x_i
                marker.pose.position.y = y_i
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = self.cell_size_x
                marker.scale.y = self.cell_size_y
                marker.scale.z = 0.01
                if self.get_cell_by_index(x, y) == self.FREE:
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 1
                elif self.get_cell_by_index(x, y) == self.OBSTACLE:
                    marker.color.r = 1
                    marker.color.g = 0
                    marker.color.b = 0
                else:
                    marker.color.r = 0
                    marker.color.g = 0
                    marker.color.b = 0


                marker.color.a = 1
                marker.lifetime = rospy.Time(0)

                markers.append(marker)

        return MarkerArray(markers)