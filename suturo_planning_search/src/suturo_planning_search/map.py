from copy import deepcopy
from math import isnan
import struct
import numpy
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Pose import Pose
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
import scipy
from sensor_msgs.msg._PointCloud2 import PointCloud2
from sensor_msgs.point_cloud2 import create_cloud_xyz32, _get_struct_fmt, read_points
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from std_msgs.msg._ColorRGBA import ColorRGBA
from suturo_perception_msgs.srv._GetPointArray import GetPointArray, GetPointArrayRequest
import time
from visualization_msgs.msg import Marker, MarkerArray
from suturo_planning_manipulation.mathemagie import *
from suturo_planning_search.cell import Cell
from suturo_planning_search.cluster_map import ClusterRegions, RegionType
from suturo_planning_visualization import visualization
from suturo_planning_manipulation.transformer import Transformer
# from suturo_perception_msgs.msg import GetPointArray
# import pcl
from suturo_msgs.msg import Task

__author__ = 'ichumuh'


class Map:
    num_of_cells = 50

    def __init__(self, size):
        # size_x = size
        # size_y = size
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]
        self.cell_size = 1.0 * size / self.num_of_cells
        # self.cell_size = 1.0 * size / len(self.field[0])
        self.size = size
        self.max_coord = self.size / 2
        # self.size = size_y
        # self.max_coord = self.size / 2
        self.obstacle_regions = []
        self.unknown_regions = []
        self.__get_point_array = rospy.ServiceProxy('/suturo/GetPointArray', GetPointArray)
        # rospy.sleep(1.0)

    def __del__(self):
        pass

    def __str__(self):
        s = "\n"
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[x])):
                if self.field[x][y].is_free():
                    s += "FF "
                elif self.field[x][y].is_object():
                    s += "OO "
                elif self.field[x][y].is_obstacle():
                    if self.field[x][y].is_blue():
                        s += "BB "
                    elif self.field[x][y].is_green():
                        s += "GG "
                    elif self.field[x][y].is_red():
                        s += "RR "
                    elif self.field[x][y].is_yellow():
                        s += "YY "
                    elif self.field[x][y].is_cyan():
                        s += "CC "
                    elif self.field[x][y].is_magenta():
                        s += "MM "
                    else:
                        s += "UU "
                elif self.field[x][y].is_unknown():
                    s += ".. "
            s += "\n"
        return s

    def print_height_map(self):
        s = ""
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[x])):
                if not self.get_cell_by_index(x, y).is_free():
                    i = int(self.get_cell_by_index(x, y).highest_z * 100)
                    if i < 10:
                        s += "0" + str(i) + " "
                    else:
                        s += str(i) + " "
                else:
                    s += "00 "
            s += "\n"
        print s

    def reset(self):
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]

    def add_point_cloud(self, arm_origin=Point(0, 0, 0), radius=0.1, scene_cam=True):
        """
        This function requests a point cloud from the scene/tcp cam and uses its data to update the map.
        The area around the arms base will be set to free.
        :param arm_origin: The centre of the arm
        :type: Point
        :param radius: The radius of the arms base
        :type: float
        :param scene_cam: True for requesting a point cloud from the scene cam, False for the tcp cam
        :type: bool
        :return: whether or not the map has changed
        :type: bool
        """

        oldmap = deepcopy(self.field)
        rospy.logdebug("scanning")

        # call odom_combiner
        request = GetPointArrayRequest()
        if scene_cam:
            request.pointCloudName = GetPointArrayRequest.SCENE
        else:
            request.pointCloudName = GetPointArrayRequest.TCP
        request.minPointCloudTimeStamp = rospy.get_rostime()

        resp = self.__get_point_array(request)
        points = resp.pointArray

        # update map with point cloud
        for i in range(0, len(points), 4):
            x = points[i]
            y = points[i + 1]
            z = points[i + 2]
            color = points[i + 3]

            if isnan(x) or x > self.max_coord or x < -self.max_coord or \
                            y > self.max_coord or y < -self.max_coord or \
                    self.__is_point_in_arm(arm_origin, radius, x, y):
                continue

            self.get_cell(x, y).update_cell(z, color)

        # Set the area around the arms base free
        for x in numpy.arange(arm_origin.x - radius, arm_origin.x + radius + 0.01, self.cell_size):
            for y in numpy.arange(arm_origin.y - radius, arm_origin.y + radius + 0.01, self.cell_size):
                if not self.get_cell(x, y).is_obstacle():
                    self.get_cell(x, y).set_free()

        for x in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size):
            for y in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size):
                self.get_cell(x, y).set_obstacle()

        # remove unknown surrounded by obstacles (or the and of the map)
        self.clean_up_map()
        self.publish_as_marker()

        rospy.logdebug("Scan complete")
        rospy.logdebug(self)
        self.print_height_map()
        return not self.field == oldmap

    def __is_point_in_arm(self, arm_origin, radius, x, y):
        return arm_origin.x - radius < x < arm_origin.x + radius and \
               arm_origin.y - radius < y < arm_origin.y + radius

    def coordinates_to_index(self, x, y):
        """
        Transforms x and y coordinates into the corresponding indices.
        :param x: x coordinate
        :type: float
        :param y: y coordinate
        :type: float
        :return: corresponding indices
        :type: (int, int)
        """
        x_index = int((x + (self.size / 2)) / self.cell_size)
        y_index = int((y + (self.size / 2)) / self.cell_size)
        if x_index >= self.num_of_cells:
            x_index = self.num_of_cells - 1

        if y_index >= self.num_of_cells:
            y_index = self.num_of_cells - 1
        return (x_index, y_index)

    def index_to_coordinates(self, x_index, y_index):
        """
        Transforms x and y indices into the corresponding coordinates.
        :param x: x index
        :type: int
        :param y: y index
        :type: int
        :return: corresponding coordinates
        :type: (float, float)
        """
        x = x_index * self.cell_size - (self.size / 2) + (self.cell_size / 2)
        y = y_index * self.cell_size - (self.size / 2) + (self.cell_size / 2)
        return (x, y)

    def clean_up_map(self):
        """
        Removes some unknowns, that are "safe" to remove
        """
        cell_changed = True
        while cell_changed:
            cell_changed = False
            for x in range(0, len(self.field)):
                for y in range(0, len(self.field[x])):
                    cell = self.get_cell_by_index(x, y)
                    if cell.is_unknown():
                        obstacles = self.get_surrounding_obstacles(x, y)
                        # remove if cells is surrounded by 3 obstacles
                        if len(obstacles) >= 3:
                            cell.set_obstacle()
                            c = obstacles[0]
                            cell.set_color(c[0].get_color_id())
                            cell_changed = True
                        # remove if cell is surrounded by 4 frees
                        elif self.is_cell_alone(x, y):
                            self.get_cell_by_index(x, y).set_free()
                            cell_changed = True
                        else:
                            free = self.get_surrounding_frees(x, y)
                            # remove if cell is surrounded by 2 obstacles and 2 frees
                            if len(free) == 2 and len(obstacles) == 2:
                                self.get_cell_by_index(x, y).set_free()
                                cell_changed = True
                    # if a obstacle is surrounded by 2 or more obstacles with an other color, change this cell color to it
                    # needed to remove some noise
                    elif cell.is_obstacle():
                        obstacles = self.get_surrounding_obstacles(x, y)
                        cell_color = cell.get_color_id()
                        obstacles_with_different_color = [c for c in obstacles if c[0].get_color_id() != cell_color]
                        if len(obstacles_with_different_color) > 2:
                            c = obstacles_with_different_color[0]
                            cell.set_color(c[0].get_color_id())
                            cell_changed = True

        self.publish_as_marker()

    def remove_puzzle_fixture(self, yaml):
        rospy.logdebug("remove_puzzle_fixture called!")
        if yaml.task_type == Task.TASK_5:
            fixture_position = deepcopy(yaml.puzzle_fixture.position)
            fixture_position = add_point(fixture_position, Point(0.115, 0.165, 0))
            for x in range(0, len(self.field)):
                for y in range(0, len(self.field[x])):
                    cell = self.get_cell_by_index(x, y)
                    cell_coor = self.index_to_coordinates(x, y)
                    fixture_dist = euclidean_distance_in_2d(fixture_position, cell_coor)
                    if fixture_dist < 0.35:
                        rospy.logdebug("cell at ("+str(x)+","+str(y)+") marked as fixture cell")
                        cell.set_obstacle()
                        cell.highest_z = 0.1
                        cell.average_z = 0.1
                        rospy.logdebug("the cell at ("+str(x)+","+str(y)+"): "+str(cell))

    def to_collision_object(self):
        '''
        :return: the map as a collision object
        :type: CollisionObject
        '''
        co = CollisionObject()
        co.header.frame_id = "/odom_combined"
        co.id = "map"
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions.append(self.cell_size)
        primitive.dimensions.append(self.cell_size)
        primitive.dimensions.append(2)

        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[x])):
                if self.field[x][y].is_free() or self.field[x][y].is_object():
                    continue
                if self.field[x][y].is_obstacle():
                    primitive.dimensions[primitive.BOX_Z] = self.get_cell_by_index(x, y).highest_z * 2
                else:
                    primitive.dimensions[primitive.BOX_Z] = self.get_average_z_of_surrounded_obstacles(x, y) * 2
                primitive.dimensions[primitive.BOX_Z] += 0.02
                co.primitives.append(deepcopy(primitive))
                pose = Pose()
                (pose.position.x, pose.position.y) = self.index_to_coordinates(x, y)
                pose.orientation.w = 1
                co.primitive_poses.append(pose)

        return co

    def mark_cell(self, x, y, marked=True):
        """
        Marks a cell orange and publishes the map.
        :param x: x coordinate
        :type float
        :param y: y coordinate
        :type float
        :param marked: whether or not the cell should be marked
        :type bool
        """
        self.get_cell(x, y).set_mark(marked)
        self.publish_as_marker()

    def all_unknowns_to_obstacle(self):
        """
        Removes all remaining unknowns and transforms them to obstacles. Should be called, after the scan is complete.
        Publishes the updated map
        """
        while True:
            unknowns = self.get_all_unknowns()
            old_l = len(unknowns)
            i = 0
            while len(unknowns) > i:
                (x, y) = unknowns[i]
                obs = self.get_surrounding_obstacles(x, y)
                # filter obstacles with undefined color
                obs = [o for o in obs if not o[0].is_undef()]
                # copy the color and height of surrounding obstacles
                if len(obs) > 0:
                    o = obs[0][0]
                    cell = self.get_cell_by_index(x, y)
                    cell.set_obstacle()
                    cell.highest_z = o.highest_z
                    cell.set_color(o.get_color_id())
                    unknowns.remove(unknowns[i])
                i += 1

            if old_l == len(unknowns):
                break

        # change the color of all remaining unknowns to undef
        for x in range(self.num_of_cells):
            for y in range(self.num_of_cells):
                cell = self.get_cell_by_index(x, y)
                if cell.is_unknown():
                    cell.set_obstacle()
                    cell.highest_z = 0.05
                    cell.set_undef()

        self.publish_as_marker()

    def is_more_edge_by_index(self, x1_index, y1_index, x2_index, y2_index):
        """
        Determines whether cell one is more edgy then cell two.
        :param x1_index: x index of the first cell
        :type: int
        :param y1_index: y index of the first cell
        :type: int
        :param x2_index: x index of the second cell
        :type: int
        :param y2_index: y index of the second cell
        :type: int
        :return: -1/0/1
        :type int
        """
        (x1, y1) = self.index_to_coordinates(x1_index, y1_index)
        (x2, y2) = self.index_to_coordinates(x2_index, y2_index)
        return self.is_more_edge(x1, y1, x2, y2)

    def is_more_edge(self, x1, y1, x2, y2):
        """
        Determines whether cell one is more edgy then cell two.
        :param x1: x coordinate of the first cell
        :type: float
        :param y1: y coordinate of the first cell
        :type: float
        :param x2: x coordinate of the second cell
        :type: float
        :param y2: y coordinate of the second cell
        :type: float
        :return: -1/0/1
        :type int
        """
        surr_c1 = self.get_surrounding_cells8(x1, y1)
        num_free1 = sum(1 for c in surr_c1 if not c[0].is_free())

        surr_c2 = self.get_surrounding_cells8(x2, y2)
        num_free2 = sum(1 for c in surr_c2 if not c[0].is_free())

        if num_free1 < num_free2:
            return 1
        elif num_free1 > num_free2:
            return -1
        else:
            num_obs1 = sum(1 for c in surr_c1 if c[0].is_obstacle())
            num_obs2 = sum(1 for c in surr_c2 if c[0].is_obstacle())
            if num_obs1 > num_obs2:
                return 1
            elif num_obs1 < num_obs2:
                return -1
            else:
                return 0

    def filter_invalid_scan_poses(self, cell_x, cell_y, pose_list):
        """
        Filters poses, that are behind obstacles or unknown which nearby the given cell.
        :param cell_x: x coordinate of the cell
        :type: float
        :param cell_y: y coordinate of the cell
        :type: float
        :param pose_list: list of poses
        :type: [PoseStamped]
        :return: filtered list of poses
        :type: [PoseStamped]
        """
        poses = deepcopy(pose_list)
        surr_cells = self.get_surrounding_cells8(cell_x, cell_y)
        for (c, x, y) in surr_cells:
            if not c.is_free():
                p = Point(x, y, 0)
                p2 = Point(cell_x, cell_y, 0)
                cell_to_p = subtract_point(p, p2)

                for pose in pose_list:
                    x_under_pose = pose.pose.position.x
                    y_under_pose = pose.pose.position.y
                    # filter poses that are pointing to unknowns or obstacles
                    cell_to_pose = subtract_point(Point(x_under_pose, y_under_pose, 0), p2)
                    angle = get_angle(cell_to_p, cell_to_pose)
                    if angle <= pi / 4 + 0.0001 and pose in poses:
                        poses.remove(pose)

        return poses

    def filter_invalid_scan_poses2(self, cell_x, cell_y, pose_list):
        """
        Filters poses, that are outside of the map, over high obstacles/unknowns or when there are high obstacles between
        the the cell and the pose.
        :param cell_x: x coordinate of the cell
        :type: float
        :param cell_y: y coordinate of the cell
        :type: float
        :param pose_list: list of poses
        :type: [PoseStamped]
        :return: filtered list of poses
        :type: [PoseStamped]
        """
        poses = deepcopy(pose_list)
        for pose in pose_list:
            x_under_pose = pose.pose.position.x
            y_under_pose = pose.pose.position.y
            # filter poses that are outside of the table
            filter_range = 1.2
            if not (-filter_range <= x_under_pose <= filter_range and -filter_range <= y_under_pose <= filter_range) \
                    and pose in poses:
                poses.remove(pose)
                continue
            # filter poses that are above unknowns
            if -1.0 <= x_under_pose <= 1.0 and -1.0 <= y_under_pose <= 1.0:
                cell = self.get_cell(x_under_pose, y_under_pose)
                if cell.is_unknown() or \
                        (cell.is_obstacle() and cell.highest_z >= pose.pose.position.z - 0.085):
                    if pose in poses: poses.remove(pose)
                    continue

                cells = self.get_surrounding_cells8(x_under_pose, y_under_pose)
                removed = False
                for (c, x, y) in cells:
                    if c.is_unknown() or (c.is_obstacle() and c.highest_z >= pose.pose.position.z - 0.085):
                        removed = True
                        if pose in poses: poses.remove(pose)
                        break
                if removed:
                    continue
                cell_between = self.get_cells_between(cell_x, cell_y, x_under_pose, y_under_pose)
                not_frees = [c[0].highest_z for c in cell_between if not c[0].is_free()]
                max_z = 0
                if len(not_frees) > 0:
                    max_z = max(not_frees)
                # filter pose if the cell to the cells are to high on average
                if max_z > pose.pose.position.z / 3.5:
                    if pose in poses: poses.remove(pose)
        return poses

    # GETTER

    def get_field(self):
        return self.field


    def get_cell(self, x, y):
        """
        :param x: x coordinate
        :type float
        :param y: y coordinate
        :type float
        :return: cell
        :type Cell
        """
        (x_index, y_index) = self.coordinates_to_index(x, y)
        return self.get_cell_by_index(x_index, y_index)

    def get_cell_by_index(self, x, y):
        """
        :param x: x index
        :type int
        :param y: y index
        :type int
        :return: cell
        :type Cell
        """
        x2 = x
        y2 = y
        warn = lambda: rospy.logwarn("Index out auf range: " + str(x) + " " + str(y))
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

    def get_surrounding_cells_by_index(self, x_index, y_index):
        """
        ..x..
        .xcx.
        ..x..
        x's are returned
        :param x_index: x index
        :type: int
        :param y_index: y index
        :type: int
        :return: list of surrounding cells
        :type: [Cell]
        """
        cells = []
        if not x_index - 1 < 0:
            cells.append((self.field[x_index - 1][y_index], x_index - 1, y_index))

        if not y_index - 1 < 0:
            cells.append((self.field[x_index][y_index - 1], x_index, y_index - 1))

        if not x_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index + 1][y_index], x_index + 1, y_index))

        if not y_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index][y_index + 1], x_index, y_index + 1))

        return cells

    def get_surrounding_cells(self, x, y):
        """
        ..x..
        .xcx.
        ..x..
        x's are returned
        :param x: x coordinate
        :type: float
        :param y: y coordinate
        :type: float
        :return: list of surrounding cells
        :type [Cell]
        """
        return self.get_surrounding_cells_by_index(*self.coordinates_to_index(x, y))

    def get_surrounding_cells8_by_index(self, x_index, y_index):
        """
        .xxx.
        .xcx.
        .xxx.
        x's are returned
        :param x_index: x index
        :type: int
        :param y_index: y index
        :type: int
        :return: list of surrounding cells
        :type: [Cell]
        """
        cells = []
        xm1 = False
        xp1 = False
        ym1 = False
        yp1 = False
        if not x_index - 1 < 0:
            cells.append((self.field[x_index - 1][y_index],) + self.index_to_coordinates(x_index - 1, y_index))
            xm1 = True

        if not y_index - 1 < 0:
            cells.append((self.field[x_index][y_index - 1],) + self.index_to_coordinates(x_index, y_index - 1))
            ym1 = True

        if not x_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index + 1][y_index],) + self.index_to_coordinates(x_index + 1, y_index))
            xp1 = True

        if not y_index + 1 >= self.num_of_cells:
            cells.append((self.field[x_index][y_index + 1],) + self.index_to_coordinates(x_index, y_index + 1))
            yp1 = True

        if xm1 and ym1:
            cells.append((self.field[x_index - 1][y_index - 1],) + self.index_to_coordinates(x_index - 1, y_index - 1))

        if xm1 and yp1:
            cells.append((self.field[x_index - 1][y_index + 1],) + self.index_to_coordinates(x_index - 1, y_index + 1))

        if xp1 and ym1:
            cells.append((self.field[x_index + 1][y_index - 1],) + self.index_to_coordinates(x_index + 1, y_index - 1))

        if xp1 and yp1:
            cells.append((self.field[x_index + 1][y_index + 1],) + self.index_to_coordinates(x_index + 1, y_index + 1))

        return cells

    def get_surrounding_cells8(self, x, y):
        """
        .xxx.
        .xcx.
        .xxx.
        x's are returned
        :param x: x coordinate
        :type: float
        :param y: y coordinate
        :type: float
        :return: list of surrounding cells
        :type [Cell]
        """
        return self.get_surrounding_cells8_by_index(*self.coordinates_to_index(x, y))

    def get_all_unknowns(self):
        """
        :return: list of the indices of all unknowns
        :type: [(int, int)]
        """
        unknowns = []
        for x in range(len(self.field)):
            for y in range(len(self.field[x])):
                if self.get_cell_by_index(x, y).is_unknown():
                    unknowns.append((x, y))
        return unknowns

    def get_average_z_of_surrounded_obstacles(self, x_index, y_index):
        """
        :param x_index: x index of the cell
        :type: int
        :param y_index: y index of the cell
        :type: int
        :return: average z
        :type float
        """
        os = self.get_surrounding_obstacles(x_index, y_index)
        z_sum = sum([c[0].highest_z for c in os])
        # (z, n) = reduce(lambda (z, n), x: (z + x[0].highest_z, n + 1) if x[0].is_obstacle() else (z, n),
        #                 self.get_surrounding_cells8_by_index(x_index, y_index), (0.0, 0.0))
        if z_sum == 0:
            return 1
        else:
            return z_sum / (len(os)+0.0)

    # def get_closest_unknown(self, arm_origin=Point(0, 0, 0)):
    #     closest_points = []
    #     for x in range(len(self.field)):
    #         for y in range(len(self.field[x])):
    #             p = Point()
    #             (p.x, p.y) = self.index_to_coordinates(x, y)
    #             if self.field[x][y].is_unknown():
    #                 sc = self.get_surrounding_cells_by_index(x, y)
    #                 if reduce(lambda free, c: free or c[0].is_free(), sc, False):
    #                     closest_points.append(p)
    #
    #     closest_points.sort(key=lambda pointx: magnitude(subtract_point(arm_origin, pointx)))
    #
    #     return closest_points

    def get_boarder_cell_points(self, region):
        """
        :param region: a region
        :type: Region
        :return: list of boarder cells of the region
        :type: [Point]
        """
        boarder_cells = []
        for [x_index, y_index] in region.cell_coords:
            p = Point()
            (p.x, p.y) = self.index_to_coordinates(x_index, y_index)
            if self.field[x_index][y_index].is_unknown():
                sc = self.get_surrounding_cells8_by_index(x_index, y_index)
                # if min 2 neighbors are free
                if sum(1 for c in sc if c[0].is_free()) > 1:
                    boarder_cells.append(p)

        return boarder_cells

    def get_percent_cleared(self):
        """
        :return: returns how much percent of the maps are not unknowns.
        :type: float 0.0 - 1.0
        """
        num_unknowns = 0.0
        for x in xrange(self.num_of_cells):
            for y in xrange(self.num_of_cells):
                if self.get_cell_by_index(x, y).is_unknown():
                    num_unknowns += 1
        percent = 1.0 - (num_unknowns / ((self.num_of_cells + 0.0) ** 2.0))
        rospy.loginfo(str(100 * percent) + " % cleared.")
        return percent

    def get_cells_between(self, x1, y1, x2, y2):
        """
        :param x1: x coordinate of the first cell
        :type: float
        :param y1: y coordinate of the first cell
        :type: float
        :param x2: x coordinate of the second cell
        :type: float
        :param y2: y coordinate of the second cell
        :type: float
        :return: a list of Cells and there coordinates that are between two given cells
        :type:[(Cell, x_index, y_index)]
        """
        #transform cells into there indices and back to coordinates to get the centre of the cell
        (x_1, y_1) = self.index_to_coordinates(*self.coordinates_to_index(x1, y1))
        (x_2, y_2) = self.index_to_coordinates(*self.coordinates_to_index(x2, y2))
        cell1_to_cell2 = subtract_point((x_2, y_2, 0), (x_1, y_1, 0))
        l = magnitude(cell1_to_cell2)
        steps = int(l / self.cell_size) + 2
        cell1_to_cell2 = set_vector_length(self.cell_size, cell1_to_cell2)
        #can be nan, when both cells are identical
        if isnan(cell1_to_cell2.x) or isnan(cell1_to_cell2.y) or isnan(cell1_to_cell2.z):
            return []
        cells = []
        x = x_1
        y = y_1
        for i in xrange(steps):
            c = self.coordinates_to_index(x, y)
            c = ((self.get_cell_by_index(*c),) + c)
            if not c in cells:
                cells.append(c)
            x += cell1_to_cell2.x
            y += cell1_to_cell2.y

        return cells

    def is_cell_alone(self, x_index, y_index):
        """
        :param x_index: x index of the cell
        :type: int
        :param y_index: y index of the cell
        :type: int
        :return: whether the cell is surrounded by 4 frees
        :type: bool
        """
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return len([c for c in sc if c[0].is_free()]) == len(sc)

    def get_surrounding_obstacles(self, x_index, y_index):
        """
        :param x_index: x index of the cell
        :type: int
        :param y_index: y index of the cell
        :type: int
        :return: list of surrounding obstacles
        :type: [Cell]
        """
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return [c for c in sc if c[0].is_obstacle()]

    def get_surrounding_frees(self, x_index, y_index):
        """
        :param x_index: x index of the cell
        :type: int
        :param y_index: y index of the cell
        :type: int
        :return: list of surrounding frees
        :type: [Cell]
        """
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return [c for c in sc if c[0].is_free()]

    # REGION SHIT

    def get_obstacle_regions(self):
        """
        Searches for a list of obstacle regions with the same color. will only be calculated once.
        :return: list of obstacle regions with the same color
        :type: [Region]
        """
        if len(self.obstacle_regions) == 0:
            cm = ClusterRegions()
            cm.set_field(self.field)

            cm.set_region_type(RegionType.blue)
            cm.group_regions()

            cm.set_region_type(RegionType.green)
            cm.group_regions()

            cm.set_region_type(RegionType.red)
            cm.group_regions()

            cm.set_region_type(RegionType.cyan)
            cm.group_regions()

            cm.set_region_type(RegionType.yellow)
            cm.group_regions()

            cm.set_region_type(RegionType.magenta)
            cm.group_regions()

            cm.set_region_type(RegionType.undef)
            cm.group_regions()

            rospy.logdebug("Clustering results:")
            rospy.logdebug(cm.print_segmented_field())
            rospy.logdebug(cm.print_field())

            self.obstacle_regions = cm.get_result_regions()
        return self.obstacle_regions

    def get_unknown_regions(self):
        """
        :return: list of unknown regions
        :type: [Region]
        """
        cm = ClusterRegions()
        cm.set_region_type(RegionType.unknown)
        cm.set_field(self.field)
        cm.group_regions()
        self.unknown_regions = cm.get_result_regions()
        return self.unknown_regions

    def get_cell_volume_by_index(self, x, y):
        """
        Calculates the volume of a cell
        :param x: x index
        :type: int
        :param y: y index
        :type: int
        :return: volume of the cell
        :type float
        """
        c = self.get_cell_by_index(x, y)
        return c.highest_z * self.cell_size * self.cell_size

    def get_region_volume(self, region):
        """
        Calculates the volume of a region.
        :param region: Region
        :return: volume of the region
        :type: float
        """
        volume = 0
        for [x, y] in region.cell_coords:
            volume += self.get_cell_volume_by_index(x, y)

        return volume

    def undercover_classifier(self, regions, objects):
        """
        This is NOT a classifier.
        :param region:
        :param yaml_objects:
        :return:
        """
        regions_copy = deepcopy(regions)
        classified_regions = []
        for o in objects:
            regions_with_same_color = [r for r in regions_copy if r.get_color_hex() == o.color]
            if len(regions_with_same_color) == 0:
                rospy.logwarn("couldnt find " + str(o.name))
            if len(regions_with_same_color) == 1:
                classified_regions.append((regions_with_same_color[0], o.name, True))
                if regions_with_same_color[0] in regions_copy:
                    regions_copy.remove(regions_with_same_color[0])
            else:
                ds = []
                for p in o.primitives:
                    if len(o.primitives) == 1 or p.type == SolidPrimitive.BOX:
                        ds.extend(p.dimensions)
                h = max(ds)
                regions_with_same_height = [r for r in regions_with_same_color if abs(r.get_height()-h) < 0.005]
                skip_classifier = len(regions_with_same_height) == 1
                for r in regions_with_same_height:
                    classified_regions.append((r, o.name, skip_classifier))
                    if r in regions_copy:
                        regions_copy.remove(r)

        return classified_regions

    def mark_region_as_object_under_point(self, x, y):
        """
        Searches for a region that contains the cell at x and y and makes every cell in it to a object.
        :param x: x coordinate
        :type: float
        :param y: y coordinate
        :type: float
        :return: true if a region was found
        :type: bool
        """
        (x_index, y_index) = self.coordinates_to_index(x, y)
        for r in self.get_obstacle_regions():
            if [x_index, y_index] in r.cell_coords:
                for coords in r.cell_coords:
                    cell = self.get_cell_by_index(*coords)
                    cell.set_object()
                    surr = self.get_surrounding_cells_by_index(*coords)
                    for c in surr:
                        c[0].set_object()
                self.publish_as_marker()
                return True
        return False

    # OTHER SHIT

    def publish_as_marker(self):
        """
        publishes the map as a marker.
        """
        visualization.publish_marker(self.to_marker())

    def __cell_to_color(self, c):
        color = ColorRGBA()
        if c.is_marked():
            color.r = 1
            color.g = 0.5
            color.b = 0
        elif c.is_free():
            color.r = 1
            color.g = 1
            color.b = 1
        elif c.is_obstacle() or c.is_object():
            if c.is_blue():
                color.r = 0
                color.g = 0
                color.b = 1
            elif c.is_green():
                color.r = 0
                color.g = 1
                color.b = 0
            elif c.is_cyan():
                color.r = 0
                color.g = 1
                color.b = 1
            elif c.is_red():
                color.r = 1
                color.g = 0
                color.b = 0
            elif c.is_magenta():
                color.r = 1
                color.g = 0
                color.b = 1
            elif c.is_yellow():
                color.r = 1
                color.g = 1
                color.b = 0
            elif c.is_undef():
                color.r = 0.5
                color.g = 0.5
                color.b = 0.5
        else:
            color.r = 0
            color.g = 0
            color.b = 0

        color.a = 1
        return color

    def to_marker(self):
        """
        :return: a marker representing the map.
        :type: Marker
        """
        marker = Marker()
        marker.type = Marker.CUBE_LIST
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[0])):
                marker.header.frame_id = '/odom_combined'
                marker.ns = 'suturo_planning/map'
                marker.id = 23
                marker.action = Marker.ADD
                (x_i, y_i) = self.index_to_coordinates(x, y)
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = self.cell_size
                marker.scale.y = self.cell_size
                marker.scale.z = 0.01

                p = Point()
                p.x = x_i
                p.y = y_i

                marker.points.append(p)
                c = self.get_cell_by_index(x, y)
                marker.colors.append(self.__cell_to_color(c))
                marker.lifetime = rospy.Time(0)

        return marker

