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

    num_free_points = 0
    num_obstacle_points = 0

    # threshold_obstacle = 0.25
    # threshold_free = 20
    average_z = 0
    points = 0

    threshold_min_points = 75

    def __init__(self):
        pass

    def __del__(self):
        pass

    def __str__(self):
        return "free: " + str(self.is_free()) +"\n" + \
               "obstacle: " + str(self.is_obstacle()) +"\n" + \
               "unknown: " + str(self.is_unknown()) +"\n" + \
               "Points: " + str(self.points) + "\n" + \
               "z: " +str(self.average_z)
               # "free points: " + str(self.num_free_points) + "\n" + \
               # "obstacle points: " + str(self.num_obstacle_points) + "\n"

    def add_point(self, z):
        z2 = self.average_z * self.points
        self.points += 1
        self.average_z = (z2 + z) / self.points
        # if 0 <= z <= 0.03:
        #     self.num_free_points += 1
        # else:
        #     self.num_obstacle_points += 1

    def set_free(self):
        self.average_z = 0
        self.points = self.threshold_min_points +1
        # self.num_free_points = self.threshold_min_points+1
        # self.num_obstacle_points = 0

    def set_obstacle(self):
        self.num_obstacle_points = self.threshold_min_points +1
        self.num_free_points = 0

    def set_unknown(self):
        self.num_free_points = 0
        self.num_obstacle_points = 0

    def is_free(self):
        return self.enough_points() and not self.is_obstacle()
        # return self.enough_points() and \
        #        not self.is_obstacle() \
        #        and self.num_free_points > (1-self.threshold_obstacle) * (self.get_num_points())

    def is_obstacle(self):
        return self.enough_points() and \
            not 0 <= self.average_z <= 0.005
               # self.num_obstacle_points > self.threshold_obstacle * (self.get_num_points())

    def is_unknown(self):
        return not self.is_obstacle() and not self.is_free()

    def enough_points(self):
        return self.points > self.threshold_min_points

    def get_num_points(self):
        return self.num_free_points + self.num_obstacle_points