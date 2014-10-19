from copy import deepcopy
import sys
from suturo_planning_search.cell import Cell
from math import sqrt

__author__ = 'pmania'


class Region:
    def get_avg(self):
        if self.avg[0] is None:
            avg_x = 0
            avg_y = 0
            for c in self.cell_coords:
                avg_x += c[0]
                avg_y += c[1]
            self.avg = [avg_x / len(self.cell_coords), avg_y / len(self.cell_coords)]

        return self.avg

    def __init__(self, rid):
        self.id = rid
        self.cells = []
        self.cell_coords = []
        self.avg = [None, None]

    def __str__(self):
        s =  "Region id: " + str(self.id) + "\n" + "Cells ("+ str(len(self.cells)) +") : " + "\n"
        for c in self.cells:
            s += str(c)
        for cc in self.cell_coords:
            s += str(cc)
        self.get_avg()
        s += " Average: " + str(self.avg[0]) + " " + str(self.avg[1])
        return s

    def euclidean_distance_to_avg(self, x1, y1):
        """
        Calculate the euclidean distance from x1, y1 to the coords that are returned
        by get_avg()"""
        x2 = self.get_avg()[0]
        y2 = self.get_avg()[1]
        return sqrt((x2 - x1) ** 2 +
                     (y2 - y1) ** 2)


class RegionType:
    obstacles = 1
    unknown = 2
    free = 3


class ClusterRegions:
    ''' This class takes a field, which is represented as a 2D-array.
        The method convert_field_to_region_map then converts this field to a region map
        where you should label every cell with SEGMENT_MAP_NOT_COLORED or
        SEGMENT_COLORED_FIELD, depending on which fields in your 2d array should be
        grouped.

        Usage:
        cm = ClusterRegions()
        cm.set_field(YOUR_2D_ARRAY_WITH_CELL_INSTANCES)
        print "Region count: " + str(cm.group_regions())
        updated_map = cm.get_result_map()
        regions = cm.get_result_regions()
    '''
    SEGMENT_MAP_NOT_COLORED = 0 # cells that should not be grouped
    SEGMENT_COLORED_FIELD = 1 # cells that should be grouped (for example obstacles)

    map_width = 25



    # Group unknown fields as default
    type = RegionType.unknown

    def set_region_type(self, type):
        '''
        Specify, which of the fields in a Cell should be grouped together as regions(unknown, obstacles, free)
        '''
        self.type = type

    def init_field(self):
        ce = Cell()
        ce.set_free()
        self.field = [[ce for x in xrange(self.map_width)] for x in xrange(self.map_width)]
        self.segmented_field = [[self.SEGMENT_MAP_NOT_COLORED for x in xrange(self.map_width)] for x in
                                xrange(self.map_width)]

    def init_segment_list(self):
        self.segments = []

    def __init__(self):
        self.init_field()
        self.regions = []
        self.result_field = [[]]

    def set_field(self, field):
        self.map_width = len(field)
        self.field = deepcopy(field)
        self.segmented_field = [[self.SEGMENT_MAP_NOT_COLORED for x in xrange(self.map_width)] for x in
                                xrange(self.map_width)]

    def print_field(self):
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[0])):
                if self.field[x][y].is_free(): sys.stdout.write(str("F"))
                if self.field[x][y].is_obstacle(): sys.stdout.write(str("O"))
                if self.field[x][y].is_unknown(): sys.stdout.write(str("U"))
            print ""

    def print_segmented_field(self):
        self.print_2d_field(self.segmented_field)


    def print_2d_field(self, field):
        for x in xrange(len(field)):
            for y in xrange(len(field[0])):
                sys.stdout.write(str(field[x][y]))
            print ""

    def fill_cell(self, x, y, color, region):
        '''color is a index for each region. should be >1'''

        # Return immediately, if we encounter a field that has been colored
        if self.segmented_field[x][y] != self.SEGMENT_COLORED_FIELD:
            return False

        # Color the field with the desired color
        self.segmented_field[x][y] = color
        self.result_field = self.field
        self.result_field[x][y].segment_id = color
        region.cells.append(self.result_field[x][y])
        # coords = []
        # coords.append(x)
        # coords.append(y)
        region.cell_coords.append([x, y])
        return True

        # Color the adjacent cells
        # if (x > 0): self.fill_cell(x - 1, y, color, region)
        # if (x < len(self.segmented_field) - 1): self.fill_cell(x + 1, y, color,region)
        # if (y > 0): self.fill_cell(x, y - 1, color, region)
        # if (y < len(self.segmented_field[0]) - 1): self.fill_cell(x, y + 1, color, region)

    def convert_field_to_region_map(self):
        ''' Turn the current self.field into a binary region map, where self.group_regions() can be performed '''
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[0])):
                self.segmented_field[x][y] = self.SEGMENT_MAP_NOT_COLORED

                # Color the cell depending on the desired region type
                if self.type == RegionType.unknown and self.field[x][y].is_unknown():
                    self.segmented_field[x][y] = self.SEGMENT_COLORED_FIELD

                if self.type == RegionType.obstacles and self.field[x][y].is_obstacle():
                    self.segmented_field[x][y] = self.SEGMENT_COLORED_FIELD

                if self.type == RegionType.free and self.field[x][y].is_free():
                    self.segmented_field[x][y] = self.SEGMENT_COLORED_FIELD



    def group_regions(self):
        self.convert_field_to_region_map()
        return self.group_regions_in_field(self.segmented_field)

    def group_regions_in_field(self, field):
        '''
        :return: the number of assigned regions
        '''
        start_color = 2
        region_color = start_color
        for x in xrange(len(field)):
            for y in xrange(len(field[0])):
                if field[x][y] == self.SEGMENT_COLORED_FIELD:
                    rx = Region(region_color)
                    maybe_cells = []
                    maybe_cells.append((x, y))
                    while len(maybe_cells) != 0:
                        (x2, y2) = maybe_cells.pop(0)
                        if self.fill_cell(x2, y2, region_color, rx):
                            maybe_cells.extend(filter(lambda (x1, y1): self.segmented_field[x1][y1] == self.SEGMENT_COLORED_FIELD,
                                                  self.get_surrounding_cells(x2, y2)))

                    self.regions.append(rx)
                    # print rx
                    region_color += 1
        return region_color - start_color

    def get_result_map(self):
        return self.result_field

    def get_result_regions(self):
        return self.regions

    def get_surrounding_cells(self, x_index, y_index):
        cells = []
        if x_index - 1 >= 0:
            cells.append((x_index-1, y_index))

        if y_index - 1 >= 0:
            cells.append((x_index, y_index-1))

        if x_index + 1 < self.map_width:
            cells.append((x_index+1, y_index))

        if y_index + 1 < self.map_width:
            cells.append((x_index, y_index+1))
        return cells

