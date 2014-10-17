import sys

__author__ = 'pmania'


def test_field1():
    field = [[0 for x in xrange(15)] for x in xrange(15)]
    field[0][6] = field[0][7] = field[0][8] = 1
    field[2][6] = field[2][7] = field[2][8] = 1
    return field


class Segment:
    id = 0
    center_coords = [0, 0]


class ClusterMap:
    SEGMENT_MAP_NOT_COLORED = 0 # cells that should not be grouped
    SEGMENT_COLORED_FIELD = 1 # cells that should be grouped (for example obstacles)

    map_width = 25

    def init_field(self):
        self.field = [[0 for x in xrange(self.map_width)] for x in xrange(self.map_width)]
        self.segmented_field = [[self.SEGMENT_MAP_NOT_COLORED for x in xrange(self.map_width)] for x in
                                xrange(self.map_width)]

    def __init__(self):
        self.init_field()

    def set_field(self, field):
        self.field = field

    def print_field(self):
        self.print_2d_field(self.field)

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
        # Color the adjacent cells
        if (x > 0): self.fill_cell(x - 1, y, color)
        if (x < len(self.segmented_field) - 1): self.fill_cell(x + 1, y, color)
        if (y > 0): self.fill_cell(x, y - 1, color)
        if (y < len(self.segmented_field[0]) - 1): self.fill_cell(x, y + 1, color)

    def convert_field_to_region_map(self):
        ''' Turn the current self.field into a binary region map, where self.group_regions() can be performed '''
        for x in xrange(len(self.field)):
            for y in xrange(len(self.field[0])):
                if self.field[x][y] == 0:
                    self.segmented_field[x][y] = self.SEGMENT_MAP_NOT_COLORED
                else:
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


cm = ClusterMap()
cm.set_field(test_field1())
cm.print_field()
cm.group_regions()
print "Grouping done"
cm.print_segmented_field()