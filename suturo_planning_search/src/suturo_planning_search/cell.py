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

__author__ = 'ichumuh'


class Cell:

    Free = 0
    Unknown = 1
    Obstacle = 2
    Object = 3

    BLUE = 255
    GREEN = 65280
    CYAN = 65535
    RED = 16711680
    MAGENTA = 16711935
    YELLOW = 16776960
    UNDEF = -1

    BLUE_ID = 0
    GREEN_ID = 1
    CYAN_ID = 2
    RED_ID = 3
    MAGENTA_ID = 4
    YELLOW_ID = 5
    UNDEF_ID = 6

    min_z_value = 0.01

    def __init__(self):
        self.average_z = 0
        self.highest_z = 0
        # self.points = 0
        self.segment_id = 0
        self.threshold_min_points = 15
        self.marked = False
        # self.object = False
        self.state = self.Unknown
        self.last_update_id = 0

        self.points = [0 for x in range(7)]

    def __del__(self):
        pass

    def __str__(self):
        return "free: " + str(self.is_free()) + \
               " obstacle: " + str(self.is_obstacle()) + \
               " unknown: " + str(self.is_unknown()) + \
               " Points: " + str(self.get_num_points()) + \
               " segment_id: " + str(self.segment_id) + \
               "z: " + str(self.average_z) + \
               "max z: " + str(self.highest_z) + "\n"

    def __eq__(self, other):
        return other.is_obstacle() and self.is_obstacle() or \
               other.is_free() and self.is_free() or \
               other.is_unknown() and self.is_unknown()

    def update_cell(self, z, color, update_id):
        self.last_update_id = update_id
        z2 = self.average_z * self.get_num_points()
        if z > self.min_z_value:
            if color == self.BLUE:
                self.points[self.BLUE_ID] += 1
            elif color == self.GREEN:
                self.points[self.GREEN_ID] += 1
            elif color == self.CYAN:
                self.points[self.CYAN_ID] += 1
            elif color == self.RED:
                self.points[self.RED_ID] += 1
            elif color == self.MAGENTA:
                self.points[self.MAGENTA_ID] += 1
            elif color == self.YELLOW:
                self.points[self.YELLOW_ID] += 1
            else:
                self.points[self.UNDEF_ID] += 1
        else:
            self.points[self.UNDEF_ID] += 1

        self.average_z = (z2 + z) / self.get_num_points()
        if z > self.highest_z:
            self.highest_z = z
        self.update_state()

    def update_state(self):
        if not self.is_object() and self.enough_points():
            if self.average_z <= self.min_z_value:
                self.state = self.Free
            else:
                self.state = self.Obstacle

    def enough_points(self):
        return self.get_num_points() > self.threshold_min_points

    def get_num_points(self):
        return sum(self.points)

    def id_to_color(self, id):
        if id == self.BLUE_ID:
            return self.BLUE
        elif id == self.GREEN_ID:
            return self.GREEN
        elif id == self.CYAN_ID:
            return self.CYAN
        elif id == self.RED_ID:
            return self.RED
        elif id == self.MAGENTA_ID:
            return self.MAGENTA
        elif id == self.YELLOW_ID:
            return self.YELLOW
        return self.UNDEF

    def color_to_id(self, id):
        if id == self.BLUE:
            return self.BLUE_ID
        elif id == self.GREEN:
            return self.GREEN_ID
        elif id == self.CYAN:
            return self.CYAN_ID
        elif id == self.RED:
            return self.RED_ID
        elif id == self.MAGENTA:
            return self.MAGENTA_ID
        elif id == self.YELLOW:
            return self.YELLOW_ID
        else:
            return self.UNDEF

    #setter

    def set_free(self):
        self.state = self.Free

    def set_object(self, is_object=True):
        self.state = self.Object

    def set_obstacle(self):
        self.state = self.Obstacle

    def set_unknown(self):
        self.state = self.Unknown

    def set_mark(self, marked=True):
        self.marked = marked

    def set_blue(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.BLUE_ID] = 50

    def set_green(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.GREEN_ID] = 50

    def set_red(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.RED_ID] = 50

    def set_yellow(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.YELLOW_ID] = 50

    def set_cyan(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.CYAN_ID] = 50

    def set_magenta(self):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[self.MAGENTA_ID] = 50

    def set_color(self, color_id):
        for x in range(len(self.points)):
            self.points[x] = 0
        self.points[color_id] = 50


    #getter

    def get_last_update(self):
        return self.last_update_id

    def get_state(self):
        return self.state

    def is_object(self):
        return self.state == self.Object

    def is_free(self):
        return self.state == self.Free

    def is_obstacle(self):
        return self.state == self.Obstacle

    def is_unknown(self):
        return self.state == self.Unknown

    def is_marked(self):
        return self.marked

    def is_updated(self):
        return self.last_update_id

    def get_color(self):
        return self.id_to_color(self.get_color_id())

    def get_color_id(self):
        id = -1
        for i in range(0, len(self.points)-1):
            if (self.points[i] > 0 and id == -1) or self.points[i] > self.points[id]:
                id = i
        if id == -1:
            return self.UNDEF_ID
        return id

    def is_blue(self):
        return (self.is_obstacle() or self.is_object()) and self.get_color_id() == self.BLUE_ID

    def is_green(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.GREEN_ID

    def is_red(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.RED_ID

    def is_cyan(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.CYAN_ID

    def is_magenta(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.MAGENTA_ID

    def is_yellow(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.YELLOW_ID

    def is_undef(self):
        return (self.is_obstacle() or self.is_object()) and  self.get_color_id() == self.UNDEF_ID
