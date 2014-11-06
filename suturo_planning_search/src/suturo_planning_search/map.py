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

__author__ = 'ichumuh'


class Map:
    num_of_cells = 50

    def __init__(self, size):
        size_x = size
        size_y = size
        self.field = [[Cell() for i in range(self.num_of_cells)] for j in range(self.num_of_cells)]
        self.cell_size_x = 1.0 * size_x / len(self.field)
        self.cell_size_y = 1.0 * size_y / len(self.field[0])
        self.size_x = size_x
        self.max_x_coord = self.size_x / 2
        self.size_y = size_y
        self.max_y_coord = self.size_y / 2
        self.obstacle_regions = []
        self.unknown_regions = []
        self.__get_point_array = rospy.ServiceProxy('/suturo/GetPointArray', GetPointArray)
        rospy.sleep(1.0)
        self.__num_of_updates = 0

    def __del__(self):
        pass

    def __str__(self):
        s = "\n"
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[x])):
                if self.field[x][y].is_free(): s += "FF "
                elif self.field[x][y].is_object(): s += "OO "
                elif self.field[x][y].is_obstacle():
                    if self.field[x][y].is_blue(): s += "BB "
                    elif self.field[x][y].is_green(): s += "GG "
                    elif self.field[x][y].is_red(): s += "RR "
                    elif self.field[x][y].is_yellow(): s += "YY "
                    elif self.field[x][y].is_cyan(): s += "CC "
                    elif self.field[x][y].is_magenta(): s += "MM "
                    else: s += "UU "
                elif self.field[x][y].is_unknown(): s += ".. "
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
        request.minPointCloudTimeStamp = rospy.get_rostime()

        # rospy.logdebug(str(rospy.get_rostime()))
        # rospy.logdebug(str(rospy.is_shutdown()))
        # rospy.logdebug(str(rospy.get_time()))
        resp = self.__get_point_array(request)
        # rospy.sleep(600)
        points = resp.pointArray

        for i in range(0, len(points), 4):
            x = points[i]
            y = points[i + 1]
            z = points[i + 2]
            color = points[i + 3]

            if isnan(x) or x > self.max_x_coord or x < -self.max_x_coord or \
                           y > self.max_y_coord or y < -self.max_y_coord or \
                           self.is_point_in_arm2(arm_origin, radius, x, y):
                continue

            self.get_cell(x, y).update_cell(z, color, self.__num_of_updates)

        #Set the area around the arms base free
        for x in numpy.arange(arm_origin.x - radius, arm_origin.x + radius + 0.01, self.cell_size_x):
            for y in numpy.arange(arm_origin.y - radius, arm_origin.y + radius + 0.01, self.cell_size_y):
                if not self.get_cell(x, y).is_obstacle():
                    self.get_cell(x, y).set_free()

        for x in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size_x):
            for y in numpy.arange(0.92 - radius, 0.92 + radius + 0.01, self.cell_size_y):
                self.get_cell(x, y).set_obstacle()

        #remove unknown surrounded by obstacles (or the and of the map)
        self.clean_up_map()
        self.publish_as_marker()

        rospy.logdebug("Scan complete")
        rospy.logdebug(self)
        self.print_height_map()
        self.__num_of_updates += 1
        return not self.field == oldmap

    def is_point_in_arm2(self, arm_origin, radius, x, y):
        return arm_origin.x - radius < x < arm_origin.x + radius and \
 	 	        arm_origin.y - radius < y < arm_origin.y + radius

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

    def clean_up_map(self):
        cell_changed = True
        while cell_changed:
            # print "changed"
            # rospy.sleep(1)
            cell_changed = False
            for x in range(0, len(self.field)):
                for y in range(0, len(self.field[x])):
                    cell = self.get_cell_by_index(x, y)
                    if cell.is_unknown():
                        obstacles = self.get_surrounding_obstacles(x, y)
                        if len(obstacles) >= 3:
                            cell.set_obstacle()
                            c = obstacles[0]
                            cell.set_color(c[0].get_color_id())
                            cell_changed = True
                        elif self.is_cell_alone(x, y):
                            self.get_cell_by_index(x, y).set_free()
                            cell_changed = True
                        else:
                            free = self.get_surrounding_frees(x, y)
                            if len(free) == 2 and len(obstacles) == 2:
                                self.get_cell_by_index(x, y).set_free()
                                cell_changed = True

                    elif cell.is_obstacle():
                        obstacles = self.get_surrounding_obstacles(x, y)
                        cell_color = cell.get_color_id()
                        obstacles_with_different_color = [c for c in obstacles if c[0].get_color_id() != cell_color]
                        if len(obstacles_with_different_color) > 2:
                            c = obstacles_with_different_color[0]
                            cell.set_color(c[0].get_color_id())
                            cell_changed = True
            # self.publish_as_marker()

        self.publish_as_marker()

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
        self.get_cell(x, y).set_mark(marked)
        self.publish_as_marker()

    def all_unknowns_to_obstacle(self):
        rospy.logdebug("removing all unknowns")
        while True:
            unknowns = self.get_all_unknowns()
            old_l = len(unknowns)
            i = 0
            while len(unknowns) > i:
                (x, y) = unknowns[i]
                obs = self.get_surrounding_obstacles(x, y)
                #filter unknown obstacles
                obs = [o for o in obs if o[0].get_color_id() != Cell.UNDEF_ID]
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

        for x in range(self.num_of_cells):
            for y in range(self.num_of_cells):
                cell = self.get_cell_by_index(x, y)
                if cell.is_unknown():
                    cell.set_obstacle()
                    # cell.highest_z = self.get_average_z_of_surrounded_obstacles(x, y)
                    # if cell.highest_z <= 0.02:
                    cell.highest_z = 0.05
                    cell.set_color(Cell.UNDEF_ID)

        self.publish_as_marker()

    def is_more_edge_by_index(self, x1_index, y1_index, x2_index, y2_index):
        (x1, y1) = self.index_to_coordinates(x1_index, y1_index)
        (x2, y2) = self.index_to_coordinates(x2_index, y2_index)
        return self.is_more_edge(x1, y1, x2, y2)

    def is_more_edge(self, x1, y1, x2, y2):
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

    def has_cell_been_updated(self, x, y):
        return self.get_cell(x, y).get_last_update() == self.__num_of_updates

    def filter_invalid_poses(self, cell_x, cell_y, pose_list):
        poses = deepcopy(pose_list)
        # poses = []
        surr_cells = self.get_surrounding_cells8(cell_x, cell_y)
        for (c, x, y) in surr_cells:
            if not c.is_free():
                p = Point(x, y, 0)
                p2 = Point(cell_x, cell_y, 0)
                cell_to_p = subtract_point(p, p2)

                for pose in pose_list:
                    x_under_pose = pose.pose.position.x
                    y_under_pose = pose.pose.position.y
                    #filter poses that are pointing to unknowns or obstacles
                    cell_to_pose = subtract_point(Point(x_under_pose, y_under_pose, 0), p2)
                    angle = get_angle(cell_to_p, cell_to_pose)
                    if angle <= pi/4+0.0001 and pose in poses:
                        poses.remove(pose)

        return poses

    def filter_invalid_poses2(self, cell_x, cell_y, pose_list):
        poses = deepcopy(pose_list)
        for pose in pose_list:
            x_under_pose = pose.pose.position.x
            y_under_pose = pose.pose.position.y
            #filter poses that are outside of the table
            filter_range = 1.2
            # print "pose"
            # print x_under_pose
            # print y_under_pose
            if not (-filter_range <= x_under_pose <= filter_range and -filter_range <= y_under_pose <= filter_range) and pose in poses:
                poses.remove(pose)
                # print "removed a"
                continue
            #filter poses that are above unknowns
            if -1.0 <= x_under_pose <= 1.0 and -1.0 <= y_under_pose <= 1.0:
                cell = self.get_cell(x_under_pose, y_under_pose)
                if cell.is_unknown() or \
                    (cell.is_obstacle() and cell.highest_z >= pose.pose.position.z - 0.085):
                    if pose in poses: poses.remove(pose)
                    # print "removed b"
                    continue

                cells = self.get_surrounding_cells8(x_under_pose, y_under_pose)
                # (x_index, y_index) = self.coordinates_to_index(x_under_pose, y_under_pose)
                # cells.append(((self.get_cell_by_index(x_index, y_index),) +(x_under_pose, y_under_pose)))
                removed = False
                for (c, x, y) in cells:
                    if c.is_unknown() or (c.is_obstacle() and c.highest_z >= pose.pose.position.z - 0.085):
                        removed = True
                        if pose in poses: poses.remove(pose)
                        # print "removed c"
                        break
                if removed:
                    continue
                cell_between = self.get_cells_between(cell_x, cell_y, x_under_pose, y_under_pose)
                not_frees = [c[0].highest_z for c in cell_between if not c[0].is_free()]
                max_z = 0
                if len(not_frees) > 0:
                    max_z = max(not_frees)
                #filter pose if the cell to the cells are to high on average
                if max_z > pose.pose.position.z/3.5:
                    # print "removed d"
                    if pose in poses: poses.remove(pose)
        return poses


    # def filter_invalid_poses3(self, cell_x, cell_y, pose_list):
    #     poses = deepcopy(pose_list)
    #     for pose in pose_list:
    #         x_under_pose = pose.pose.position.x
    #         y_under_pose = pose.pose.position.y
    #         #filter poses that are outside of the table
    #         filter_range = 1.2
    #         # print "pose"
    #         # print x_under_pose
    #         # print y_under_pose
    #         if not (-filter_range <= x_under_pose <= filter_range and -filter_range <= y_under_pose <= filter_range) and pose in poses:
    #             poses.remove(pose)
    #             # print "removed a"
    #             continue
    #         #filter poses that are above unknowns
    #         if -1.0 <= x_under_pose <= 1.0 and -1.0 <= y_under_pose <= 1.0:
    #             cell = self.get_cell(x_under_pose, y_under_pose)
    #             if cell.is_unknown() or \
    #                 (cell.is_obstacle() and cell.highest_z >= pose.pose.position.z - 0.085):
    #                 if pose in poses: poses.remove(pose)
    #                 # print "removed b"
    #                 continue
    #
    #             cells = self.get_surrounding_cells8(x_under_pose, y_under_pose)
    #             # (x_index, y_index) = self.coordinates_to_index(x_under_pose, y_under_pose)
    #             # cells.append(((self.get_cell_by_index(x_index, y_index),) +(x_under_pose, y_under_pose)))
    #             removed = False
    #             for (c, x, y) in cells:
    #                 if c.is_unknown() or (c.is_obstacle() and c.highest_z >= pose.pose.position.z - 0.085):
    #                     removed = True
    #                     if pose in poses: poses.remove(pose)
    #                     # print "removed c"
    #                     break
    #             if removed:
    #                 continue
    #             cell_between = self.get_cells_between(cell_x, cell_y, x_under_pose, y_under_pose)
    #             not_frees = [c[0].highest_z for c in cell_between if not c[0].is_free()]
    #             max_z = 0
    #             if len(not_frees) > 0:
    #                 max_z = max(not_frees)
    #             #filter pose if the cell to the cells are to high on average
    #             if max_z > pose.pose.position.z/3.5:
    #                 # print "removed d"
    #                 if pose in poses: poses.remove(pose)
    #     return poses

    #GETTER

    def get_field(self):
        return self.field


    def get_cell(self, x, y):

        (x_index, y_index) = self.coordinates_to_index(x, y)
        return self.get_cell_by_index(x_index, y_index)

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
        return self.get_surrounding_cells_by_index(*self.coordinates_to_index(x,y))

    def get_surrounding_cells8_by_index(self, x_index, y_index):
        cells = []
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

    def get_surrounding_cells8(self, x, y):
        return self.get_surrounding_cells8_by_index(*self.coordinates_to_index(x,y))

    def get_all_unknowns(self):
        unknowns = []
        for x in range(len(self.field)):
            for y in range(len(self.field[x])):
                if self.get_cell_by_index(x,y).is_unknown():
                    unknowns.append((x, y))
        return unknowns

    def get_average_z_of_surrounded_obstacles(self, x_index, y_index):

        (z, n) = reduce(lambda (z, n), x: (z + x[0].highest_z, n+1) if x[0].is_obstacle() else (z,n), self.get_surrounding_cells8_by_index(x_index, y_index), (0.0, 0.0))
        if z == 0:
            return 1
        else:
            return z / n

    def get_closest_unknown(self, arm_origin=Point(0, 0, 0)):
        closest_points = []
        for x in range(len(self.field)):
            for y in range(len(self.field[x])):
                p = Point()
                (p.x, p.y) = self.index_to_coordinates(x, y)
                if self.field[x][y].is_unknown():
                    sc = self.get_surrounding_cells_by_index(x, y)
                    if reduce(lambda free, c: free or c[0].is_free(), sc, False):
                        closest_points.append(p)

        closest_points.sort(key=lambda pointx: magnitude(subtract_point(arm_origin, pointx)))

        return closest_points

    def get_boarder_cells_points(self, region):
        boarder_cells = []
        for [x_index, y_index] in region.cell_coords:
            p = Point()
            (p.x, p.y) = self.index_to_coordinates(x_index, y_index)
            if self.field[x_index][y_index].is_unknown():
                sc = self.get_surrounding_cells8_by_index(x_index, y_index)
                #if min 2 neighbors are free
                if sum(1 for c in sc if c[0].is_free()) > 1:
                    boarder_cells.append(p)

        return boarder_cells

    def get_percent_cleared(self):
        num_unknowns = 0.0
        for x in xrange(self.num_of_cells):
            for y in xrange(self.num_of_cells):
                if self.get_cell_by_index(x, y).is_unknown():
                    num_unknowns += 1
        percent = 1.0 - (num_unknowns / ((self.num_of_cells+0.0)**2.0))
        rospy.loginfo(str(100 * percent) + " % cleared.")
        return percent

    def get_cells_between(self, x1, y1, x2, y2):
        '''

        :param x1:
        :param y1:
        :param x2:
        :param y2:
        :return: [(x_index, y_index)] list of cell coordinates
        '''

        (x_1, y_1) = self.index_to_coordinates(*self.coordinates_to_index(x1, y1))
        (x_2, y_2) = self.index_to_coordinates(*self.coordinates_to_index(x2, y2))
        p = subtract_point((x_2, y_2, 0), (x_1, y_1, 0))
        l = magnitude(p)
        steps = int(l / self.cell_size_x)+2
        p = set_vector_length(self.cell_size_x, p)
        if isnan(p.x) or isnan(p.y) or isnan(p.z):
            return []
        cells = []
        x = x_1
        y = y_1
        for i in xrange(steps):
            c = self.coordinates_to_index(x, y)
            c = ((self.get_cell_by_index(*c),) + c)
            if not c in cells:
                cells.append(c)
            x += p.x
            y += p.y

        return cells

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

    def is_cell_alone(self, x_index, y_index):
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return len([c for c in sc if c[0].is_free()]) == len(sc)

    def get_surrounding_obstacles(self, x_index, y_index):
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return [c for c in sc if c[0].is_obstacle()]

    def get_surrounding_frees(self, x_index, y_index):
        sc = self.get_surrounding_cells_by_index(x_index, y_index)
        return [c for c in sc if c[0].is_free()]


    # def is_cell_surrounded_by_obstacles(self, x_index, y_index):
        # sc = self.get_surrounding_cells_by_index(x_index, y_index)
        # return len(self.get_surrounding_obstacles(x_index, y_index)) >= 4
        # return len(sc)-1 <= reduce(lambda num_of_obstacles, c: num_of_obstacles + 1  if c[0].is_obstacle() else num_of_obstacles, sc, 0)

    #REGION SHIT

    def get_obstacle_regions(self):
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
        # if len(self.unknown_regions) == 0:
        cm = ClusterRegions()
        cm.set_region_type(RegionType.unknown)
        cm.set_field(self.field)
        cm.group_regions()
        self.unknown_regions = cm.get_result_regions()
        return self.unknown_regions

    def get_cell_volume_by_index(self, x, y):
        c = self.get_cell_by_index(x, y)
        return c.highest_z * self.cell_size_x * self.cell_size_y

    def get_region_volume(self, region):
        volume = 0
        for [x, y] in region.cell_coords:
            volume += self.get_cell_volume_by_index(x, y)

        return volume

    def mark_region_as_object_under_point(self, x, y):
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

    #OTHER SHIT

    def publish_as_marker(self):
        visualization.publish_marker(self.to_marker_array())

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

    def to_marker_array(self):
        # markers = []
        marker = Marker()
        marker.type = Marker.CUBE_LIST
        for x in range(0, len(self.field)):
            for y in range(0, len(self.field[0])):
                # marker = Marker()

                # marker.header.stamp = r
                marker.header.frame_id = '/odom_combined'
                marker.ns = 'suturo_planning/map'
                marker.id = 23
                marker.action = Marker.ADD
                # marker.type = Marker.CUBE
                (x_i, y_i) = self.index_to_coordinates(x, y)
                marker.pose.position.x = 0
                marker.pose.position.y = 0
                marker.pose.position.z = 0
                marker.pose.orientation.w = 1
                marker.scale.x = self.cell_size_x
                marker.scale.y = self.cell_size_y
                marker.scale.z = 0.01

                p = Point()
                p.x = x_i
                p.y = y_i

                marker.points.append(p)

                c = self.get_cell_by_index(x, y)

                marker.colors.append(self.__cell_to_color(c))

                marker.lifetime = rospy.Time(0)

                # markers.append(marker)

        return marker

