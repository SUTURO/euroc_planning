from copy import deepcopy
import sys
from suturo_planning_search.cell import Cell

__author__ = 'pmania'


def test_field1():
    c = Cell()
    c.set_free()
    field = [[c for x in xrange(15)] for x in xrange(15)]

    unknown_cell = Cell()
    unknown_cell.set_unknown()
    field[0][6] = field[0][7] = field[0][8] = unknown_cell
    field[2][6] = field[2][7] = field[2][8] = unknown_cell
    return field


class Segment:
    id = 0

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
    '''
    SEGMENT_MAP_NOT_COLORED = 0 # cells that should not be grouped
    SEGMENT_COLORED_FIELD = 1 # cells that should be grouped (for example obstacles)

    map_width = 25
    segments = []
    result_field = [[]]

    # Group unknown fields as default
    type = RegionType.unknown

    def set_region_type(self, type):
        '''
        Specify, which of the fields in a Cell should be grouped together as regions(unknown, obstacles, free)
        '''
        self.type = type

    def init_field(self):
        c = Cell()
        c.set_free()
        self.field = [[c for x in xrange(self.map_width)] for x in xrange(self.map_width)]
        self.segmented_field = [[self.SEGMENT_MAP_NOT_COLORED for x in xrange(self.map_width)] for x in
                                xrange(self.map_width)]

    def init_segment_list(self):
        self.segments = []

    def __init__(self):
        self.init_field()

    def set_field(self, field):
        self.field = deepcopy(field)

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

    def fill_cell(self, x, y, color):
        '''color is a index for each region. should be >0'''

        # Return immediately, if we encounter a field that has been colored
        if self.segmented_field[x][y] != self.SEGMENT_COLORED_FIELD:
            return

        # Color the field with the desired color
        self.segmented_field[x][y] = color
        self.result_field = self.field
        self.result_field[x][y].segment_id = color

        # Color the adjacent cells
        if (x > 0): self.fill_cell(x - 1, y, color)
        if (x < len(self.segmented_field) - 1): self.fill_cell(x + 1, y, color)
        if (y > 0): self.fill_cell(x, y - 1, color)
        if (y < len(self.segmented_field[0]) - 1): self.fill_cell(x, y + 1, color)

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
                if (field[x][y] == self.SEGMENT_COLORED_FIELD):
                    self.fill_cell(x, y, region_color)
                    region_color += 1
        return region_color - start_color

    def get_result_map(self):
        return self.result_field


cm = ClusterRegions()
cm.set_field(test_field1())
cm.print_field()

print "Returned regions: " + str(cm.group_regions())
print "Grouping done"
cm.print_segmented_field()
c = cm.get_result_map()