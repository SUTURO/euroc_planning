from copy import deepcopy
from math import isnan
import struct
import numpy
import rospy
import scipy
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, _get_struct_fmt, read_points
from suturo_perception_msgs.srv._GetPointArray import GetPointArray, GetPointArrayRequest
from visualization_msgs.msg import Marker, MarkerArray
from suturo_planning_visualization import visualization
from suturo_planning_manipulation.transformer import Transformer
# from suturo_perception_msgs.msg import GetPointArray
# import pcl

__author__ = 'ichumuh'


class Cell:
    def __init__(self):
        self.num_free_points = 0
        self.num_obstacle_points = 0
        self.average_z = 0
        self.highest_z = 0
        self.points = 0
        self.segment_id = 0
        self.threshold_min_points = 75

    def __del__(self):
        pass

    def __str__(self):
        return "free: " + str(self.is_free()) + \
               " obstacle: " + str(self.is_obstacle()) + \
               " unknown: " + str(self.is_unknown()) + \
               " Points: " + str(self.points) + \
               " segment_id: " + str(self.segment_id) + \
               "z: " + str(self.average_z) + "\n"
        # "free points: " + str(self.num_free_points) + "\n" + \
        # "obstacle points: " + str(self.num_obstacle_points) + "\n"

    def add_point(self, z):
        z2 = self.average_z * self.points
        self.points += 1
        self.average_z = (z2 + z) / self.points
        if z > self.highest_z:
            self.highest_z = z

    def set_free(self):
        self.average_z = 0
        self.points = self.threshold_min_points + 1

    def set_obstacle(self):
        self.num_obstacle_points = self.threshold_min_points + 1
        self.num_free_points = 0

    def set_unknown(self):
        self.num_free_points = 0
        self.num_obstacle_points = 0

    def is_free(self):
        return self.enough_points() and not self.is_obstacle()

    def is_obstacle(self):
        return self.enough_points() and \
               not 0 <= self.average_z <= 0.01

    def is_unknown(self):
        return not self.is_obstacle() and not self.is_free()

    def enough_points(self):
        return self.points > self.threshold_min_points

    def get_num_points(self):
        return self.num_free_points + self.num_obstacle_points

    def __eq__(self, other):
        return other.is_obstacle() and self.is_obstacle() or \
               other.is_free() and self.is_free() or \
               other.is_unknown() and self.is_unknown()
