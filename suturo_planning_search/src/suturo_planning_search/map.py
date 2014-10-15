from copy import deepcopy
from math import isnan
import struct
import numpy
# from datetime import time
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Pose import Pose
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
import scipy
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, _get_struct_fmt, read_points
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from suturo_perception_msgs.srv._GetPointArray import GetPointArray, GetPointArrayRequest
import time
from visualization_msgs.msg import Marker, MarkerArray
from mathemagie import magnitude
from suturo_planning_search.Cell import Cell
from suturo_planning_visualization import visualization
from transformer import Transformer
# from suturo_perception_msgs.msg import GetPointArray
# import pcl

__author__ = 'ichumuh'


class Map:

    num_of_cells = 25

    def __init__(self, size_x, size_y):
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]
        # self.field = scipy.zeros([num_of_fields_x, num_of_fields_y])
        self.cell_size_x = 1.0 * size_x / len(self.field)
        self.cell_size_y = 1.0 * size_y / len(self.field[0])
        self.size_x = size_x
        self.max_x_coord = self.size_x /2
        self.size_y = size_y
        self.max_y_coord = self.size_y /2
        # self.tf = Transformer()
        # self.tcp_cloud = None

        # rospy.Subscriber("/suturo/euroc_scene_cloud", PointCloud2, self.tcp_callback)
        # self.coordinates = self._get_real_coordinates(size_x, size_y)
        self.__get_point_array = rospy.ServiceProxy('/suturo/GetPointArray', GetPointArray)
        rospy.sleep(1.0)

    def __del__(self):
        pass

    def reset(self):
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]

    def tcp_callback(self, data):
        self.tcp_cloud = data

    def add_point_cloud(self, arm_origin=Point(0, 0, 0), radius=0.1, scene_cam=True):
        request = GetPointArrayRequest()
        if scene_cam:
            request.pointCloudName = GetPointArrayRequest.SCENE
        else:
            request.pointCloudName = GetPointArrayRequest.TCP

        resp = self.__get_point_array(request)
        points = resp.pointArray
        print "start"
        start = time.time()

        for i in range(0, len(points), 3):
            x = points[i]
            y = points[i+1]
            z = points[i+2]

            if isnan(x) or isnan(y) or \
                    x > self.max_x_coord or x < -self.max_x_coord or \
                    y > self.max_y_coord or y < -self.max_y_coord or \
                    self.is_point_in_arm2(arm_origin, radius, x, y):
                continue

            # if self.is_point_in_arm2(arm_origin, radius, x, y):
            #     self.get_cell(x,y).set_free()
            #     continue


            self.get_cell(x,y).add_point(z)

        for x in numpy.arange(arm_origin.x - radius, arm_origin.x + radius+0.01, self.cell_size_x):
            for y in numpy.arange(arm_origin.y - radius, arm_origin.y + radius+0.01, self.cell_size_y):
                self.get_cell(x,y).set_free()
                # print x

        print time.time() - start

    def is_point_in_arm(self, arm_origin, radius, x, y):
        dist_x = arm_origin.x - x
        dist_y = arm_origin.y - y
        return magnitude(Point(dist_x, dist_y, 0)) < radius

    def is_point_in_arm2(self, arm_origin, radius, x, y):
        return arm_origin.x - radius < x < arm_origin.x + radius and \
               arm_origin.y - radius < y < arm_origin.y + radius

    def get_cell(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x, y)
        return self.get_cell_by_index(x_index, y_index)

    def get_cell_by_index(self, x, y):
        return self.field[x][y]

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
        return (x, y)

    def publish_as_marker(self):
        visualization.publish_marker_array(self.to_marker_array())

    def get_collision_objects(self):
        cos = []
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                if self.field[x][y].is_free():
                    continue
                co = CollisionObject()
                co.id = "block_" + str(x) + "_" + str(y)
                co.header.frame_id = "/odom_combined"
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions.append(self.cell_size_x)
                primitive.dimensions.append(self.cell_size_y)
                primitive.dimensions.append(2)
                co.primitives.append(primitive)

                pose = Pose()
                (pose.position.x, pose.position.y) = self.index_to_coordinates(x,y)
                pose.orientation.w = 1
                co.primitive_poses.append(pose)
                cos.append(co)

        return cos

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
                if self.get_cell_by_index(x, y).is_free():
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 1
                elif self.get_cell_by_index(x, y).is_obstacle():
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

