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
from suturo_planning_manipulation.mathemagie import *
from suturo_planning_search.cell import Cell
from suturo_planning_search.cluster_map import ClusterRegions
from suturo_planning_visualization import visualization
from suturo_planning_manipulation.transformer import Transformer
# from suturo_perception_msgs.msg import GetPointArray
# import pcl

__author__ = 'ichumuh'


class Map:
    num_of_cells = 50

    def __init__(self, size_x, size_y):
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]
        self.cell_size_x = 1.0 * size_x / len(self.field)
        self.cell_size_y = 1.0 * size_y / len(self.field[0])
        self.size_x = size_x
        self.max_x_coord = self.size_x / 2
        self.size_y = size_y
        self.max_y_coord = self.size_y / 2
        self.__get_point_array = rospy.ServiceProxy('/suturo/GetPointArray', GetPointArray)
        rospy.sleep(1.0)

    def __del__(self):
        pass

    def __str__(self):
        s = ""
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                s = s + str("f" if self.field[x][y].is_free() else "o" if self.field[x][y].is_obstacle() else " ")
            s = s + "\n"
        return s

    def reset(self):
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]

    def add_point_cloud(self, arm_origin=Point(0, 0, 0), radius=0.1, scene_cam=True):
        '''
        This function requests a pointcloud from the scene/tcp cam and uses its data to update the map.
        The area around the arms base will be set to free.
        :param arm_origin: The centre of the arm
        :param radius: The radius of the arms base
        :param scene_cam: True for requesting a pointcloud from the scene cam, False for the tcp cam
        :return: True if the map has changed
        '''

        oldmap = deepcopy(self.field)
        rospy.logdebug("scanning")
        request = GetPointArrayRequest()
        if scene_cam:
            request.pointCloudName = GetPointArrayRequest.SCENE
        else:
            request.pointCloudName = GetPointArrayRequest.TCP

        # request.publishToPlanningScene = True
        resp = self.__get_point_array(request)
        points = resp.pointArray

        for i in range(0, len(points), 3):
            x = points[i]
            y = points[i + 1]
            z = points[i + 2]

            if isnan(x) or \
                            x > self.max_x_coord or x < -self.max_x_coord or \
                            y > self.max_y_coord or y < -self.max_y_coord or \
                    self.is_point_in_arm2(arm_origin, radius, x, y):
                continue

            self.get_cell(x, y).add_point(z)

        #Set the area around the arms base free
        for x in numpy.arange(arm_origin.x - radius, arm_origin.x + radius + 0.01, self.cell_size_x):
            for y in numpy.arange(arm_origin.y - radius, arm_origin.y + radius + 0.01, self.cell_size_y):
                if not self.get_cell(x, y).is_obstacle():
                    self.get_cell(x, y).set_free()

        for x in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size_x):
            for y in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size_y):
                self.get_cell(x, y).set_obstacle()

        #remove unknown surrounded by obstacles (or the and of the map)
        self.remove_unreachable_unknowns()
        self.publish_as_marker()
        return not self.field == oldmap

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
        x2 = x
        y2 = y
        warn = lambda : rospy.logwarn("Index out auf range: " + str(x) + " " + str(y))
        if x < 0:
            warn()
            x2 = 0
        if y < 0:
            warn()
            y2 = 0
        if x >= len(self.field):
            warn()
            x2 = len(self.field) - 1
        if y >= len(self.field):
            warn()
            y2 = len(self.field) - 1

        return self.field[x2][y2]

    def coordinates_to_index(self, x, y):
        x_index = int((x + (self.size_x / 2)) / self.cell_size_x)
        y_index = int((y + (self.size_y / 2)) / self.cell_size_y)
        if x_index >= self.num_of_cells:
            x_index = self.num_of_cells -1

        if y_index >= self.num_of_cells:
            y_index = self.num_of_cells -1
        return (x_index, y_index)

    def index_to_coordinates(self, x_index, y_index):
        x = x_index * self.cell_size_x - (self.size_x / 2) + (self.cell_size_x / 2)
        y = y_index * self.cell_size_y - (self.size_y / 2) + (self.cell_size_y / 2)
        return (x, y)

    def publish_as_marker(self):
        visualization.publish_marker_array(self.to_marker_array())

    def remove_unreachable_unknowns(self):
        cell_changed = True
        while cell_changed:
            cell_changed = False
            for x in range(0, len(self.field)):
                for y in range(0, len(self.field[x])):
                    if self.get_cell_by_index(x, y).is_unknown() and self.is_cell_surrounded_by_obstacles(x, y):
                        self.get_cell_by_index(x, y).set_obstacle()
                        cell_changed = True

    def is_cell_surrounded_by_obstacles(self, x_index, y_index):
        yes = x_index - 1 < 0 or self.get_cell_by_index(x_index - 1, y_index).is_obstacle()

        yes = yes and (y_index - 1 < 0 or self.get_cell_by_index(x_index, y_index - 1).is_obstacle())

        yes = yes and (x_index + 1 >= self.num_of_cells or self.get_cell_by_index(x_index + 1, y_index).is_obstacle())

        yes = yes and (y_index + 1 >= self.num_of_cells or self.get_cell_by_index(x_index, y_index + 1).is_obstacle())

        return yes

    def to_collision_object(self):
        '''
        :return: the map as a CollisionObject
        '''
        co = CollisionObject()
        co.header.frame_id = "/odom_combined"
        co.id = "map"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions.append(self.cell_size_x)
        primitive.dimensions.append(self.cell_size_y)
        primitive.dimensions.append(2)

        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                if self.field[x][y].is_free():
                    continue
                if self.field[x][y].is_obstacle():
                    primitive.dimensions[primitive.BOX_Z] = self.get_cell_by_index(x, y).highest_z * 2
                else:
                    primitive.dimensions[primitive.BOX_Z] = self.get_average_z_of_surrounded_obstacles(x, y) * 2
                co.primitives.append(deepcopy(primitive))
                pose = Pose()
                (pose.position.x, pose.position.y) = self.index_to_coordinates(x, y)
                pose.orientation.w = 1
                co.primitive_poses.append(pose)

        return co

    def get_average_z_of_surrounded_obstacles(self, x_index, y_index):
        z = 0
        n = 0
        if not x_index - 1 < 0 and self.get_cell_by_index(x_index - 1, y_index).is_obstacle():
            z += self.get_cell_by_index(x_index - 1, y_index).highest_z
            n += 1

        if not y_index - 1 < 0 and self.get_cell_by_index(x_index, y_index - 1).is_obstacle():
            z += self.get_cell_by_index(x_index, y_index - 1).highest_z
            n += 1

        if not x_index + 1 >= self.num_of_cells and self.get_cell_by_index(x_index + 1, y_index).is_obstacle():
            z += self.get_cell_by_index(x_index + 1, y_index).highest_z
            n += 1

        if not y_index + 1 >= self.num_of_cells and self.get_cell_by_index(x_index, y_index + 1).is_obstacle():
            z += self.get_cell_by_index(x_index, y_index + 1).highest_z
            n += 1

        if z == 0:
            return 1
        else:
            return z / n

    def get_closest_unknown(self, arm_origin=Point(0, 0, 0)):
        closest_points = []
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                p = Point()
                (p.x, p.y) = self.index_to_coordinates(x, y)
                if self.field[x][y].is_unknown():
                    sc = self.get_surrounding_cells_by_index(x, y)
                    if reduce(lambda free, c: free or c[0].is_free(), sc, False):
                        closest_points.append(p)

        closest_points.sort(key=lambda pointx: magnitude(subtract_point(arm_origin, pointx)))

        return closest_points


    def get_cell_above(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x,y)
        return None if x_index-1 < 0 else self.get_cell_by_index(x_index-1, y_index)

    def get_cell_below(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x,y)
        return None if x_index+1 >= self.size_x else self.get_cell_by_index(x_index+1, y_index)

    def get_cell_left(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x,y)
        return None if y_index-1 < 0 else self.get_cell_by_index(x_index, y_index-1)

    def get_cell_right(self, x, y):
        (x_index, y_index) = self.coordinates_to_index(x,y)
        return None if y_index+1 >= self.size_y else self.get_cell_by_index(x_index, y_index+1)

    def get_next_point(self, arm_x = 0, arm_y = 0):
        cm = ClusterRegions()
        cm.set_field(self.field)
        cm.group_regions()
        regions = cm.get_result_regions()
        if len(regions) == 0:
            return None
        (ax, ay) = self.coordinates_to_index(arm_x, arm_y)
        next_r = min(regions, key=lambda r: r.euclidean_distance_to_avg(ax, ay))

        boarder_cells = []
        for i in range(len(next_r.cells)):
            sc = self.get_surrounding_cells_by_index(*next_r.cell_coords[i])
            # sc = map(lambda (c, x, y): c.is_free(), sc)
            if reduce(lambda free, (c, x, y): free or c.is_free, sc, False):
                boarder_cells.append((next_r.cells, next_r.cell_coords[i][0], next_r.cell_coords[i][1]))

        # boarder_cells = map(lambda  (c, x, y): self.index_to_coordinates(x, y), boarder_cells)
        return map(lambda bc: Point(*self.index_to_coordinates(*bc[1:3])+(0,)), boarder_cells), Point(*self.index_to_coordinates(*tuple(next_r.get_avg()))+(0,))
        # (closest_boarder_cell, cx, cy) = min(boarder_cells, key=lambda (c, x, y): euclidean_distance(Point(ax, ay, 0), Point(x, y, 0)))
        # p = Point()
        # (p.x, p.y) = self.index_to_coordinates(cx, cy)
        # return p

    def mark_cell(self, x, y, marked=True):
        self.get_cell(x, y).set_mark(marked)
        self.publish_as_marker()

    def get_surrounding_cells_by_index(self, x_index, y_index):
        cells = []
        if not x_index - 1 < 0:
            cells.append((self.field[x_index-1][y_index], x_index-1, y_index))

        if not y_index - 1 < 0:
            cells.append((self.field[x_index][y_index-1], x_index, y_index-1))

        if not x_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index+1][y_index], x_index+1, y_index))

        if not y_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index][y_index+1], x_index, y_index+1))

        return cells

    def get_surrounding_cells(self, x, y):
        cells = []
        (x_index, y_index) = self.coordinates_to_index(x,y)
        if not x_index - 1 < 0:
            cells.append((self.field[x_index-1][y_index],) + self.index_to_coordinates(x_index-1, y_index))

        if not y_index - 1 < 0:
            cells.append((self.field[x_index][y_index-1],) + self.index_to_coordinates(x_index, y_index-1))

        if not x_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index+1][y_index],) + self.index_to_coordinates(x_index+1, y_index))

        if not y_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index][y_index+1],) + self.index_to_coordinates(x_index, y_index+1))

        return cells

    def get_surrounding_cells8(self, x, y):
        cells = []
        (x_index, y_index) = self.coordinates_to_index(x,y)
        xm1 = False
        xp1 = False
        ym1 = False
        yp1 = False
        if not x_index - 1 < 0:
            cells.append((self.field[x_index-1][y_index],) + self.index_to_coordinates(x_index-1, y_index))
            xm1 = True

        if not y_index - 1 < 0:
            cells.append((self.field[x_index][y_index-1],) + self.index_to_coordinates(x_index, y_index-1))
            ym1 = True

        if not x_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index+1][y_index],) + self.index_to_coordinates(x_index+1, y_index))
            xp1 = True

        if not y_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index][y_index+1],) + self.index_to_coordinates(x_index, y_index+1))
            yp1 = True

        if xm1 and ym1:
            cells.append((self.field[x_index-1][y_index-1],) + self.index_to_coordinates(x_index-1, y_index-1))

        if xm1 and yp1:
            cells.append((self.field[x_index-1][y_index+1],) + self.index_to_coordinates(x_index-1, y_index+1))

        if xp1 and ym1:
            cells.append((self.field[x_index+1][y_index-1],) + self.index_to_coordinates(x_index+1, y_index-1))

        if xp1 and yp1:
            cells.append((self.field[x_index+1][y_index+1],) + self.index_to_coordinates(x_index+1, y_index+1))


        return cells

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
                c = self.get_cell_by_index(x, y)
                if c.is_marked():
                    marker.color.r = 0
                    marker.color.g = 0
                    marker.color.b = 1
                elif c.is_free():
                    marker.color.r = 1
                    marker.color.g = 1
                    marker.color.b = 1
                elif c.is_obstacle():
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

