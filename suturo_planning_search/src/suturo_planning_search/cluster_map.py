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
        self.min_y = None
        self.max_y = 0
        self.min_x = None
        self.max_x = 0

    def __str__(self):
        s =  "Region id: " + str(self.id) + "\n" + "Cells ("+ str(len(self.cells)) +") : " + "\n"
        for c in self.cells:
            s += str(c)
        for cc in self.cell_coords:
            s += str(cc)
        self.get_avg()
        s += "\nAverage: " + str(self.avg[0]) + " " + str(self.avg[1])
        s += "\nmin/max dims: (" + str(self.min_x) + "x" + str(self.min_y) + ")-(" + str(self.max_x) + "x" + str(self.max_y) + ")"
        return s

    def euclidean_distance_to_avg(self, x1, y1):
        """
        Calculate the euclidean distance from x1, y1 to the coords that are returned
        by get_avg()"""
        x2 = self.get_avg()[0]
        y2 = self.get_avg()[1]
        return sqrt((x2 - x1) ** 2 +
                     (y2 - y1) ** 2)

    def get_number_of_cells(self):
        return len(self.cells)

    def merge_region(self, other_region):
        """ Copy the cells of other_region with self.cells"""
        self.cells.extend(other_region.cells)
        self.cell_coords.extend(other_region.cell_coords)
        self.get_avg()
        if self.min_x > other_region.min_x:
            self.min_x = other_region.min_x
        if self.min_y > other_region.min_y:
            self.min_y = other_region.min_y
        if self.max_x < other_region.max_x:
            self.max_x = other_region.max_x
        if self.max_y < other_region.max_y:
            self.max_y = other_region.max_y

    def merge_region_w_criteria(self, spacing_left, spacing_right, other_region):
        """" Merges a region, if abs(self.min_x - other_region.min_x) <= spacing_left
             and abs(self.max_x - other_region.max) <= spacing_right
             Returns false otherwise"""
        if abs(self.min_y - other_region.min_y) <= spacing_left and abs(self.max_y - other_region.max_y) <= spacing_right:
            self.merge_region(other_region)
            return True
        else:
            return False

class RegionType:
    obstacles = 1
    unknown = 2
    free = 3

class RegionLineFragment:

    def __init__(self):
        self.cells = []
        self.cell_coords = []
        self.min_y = None
        self.max_y = 0
        self.x = None

    def __str__(self):
        s = "RegionLineFragment - min_y: " + str(self.min_y) + ", max_y: " + str(self.max_y) + ", x: " + str(self.x) + " "
        for cc in self.cell_coords:
           s += str(cc)
        return s

    def to_region(self,rid):
        r = Region(rid)
        r.cells = self.cells
        r.cell_coords = self.cell_coords
        r.min_x = self.x
        r.max_x = self.x
        r.min_y = self.min_y
        r.max_y = self.max_y
        return r


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
    REGION_SPLIT_SPACE_TRESHOLD = 0 # How many adjacent cells can be empty but it still forms a line?

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
        # fill min/max values in region
        if(x > region.max_x): region.max_x = x
        if(y > region.max_y): region.max_y = y
        if(x < region.min_x or region.min_x is None): region.min_x = x
        if(y < region.min_y or region.min_y is None): region.min_y = y
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
                                                  self.get_surrounding_cells8(x2, y2)))

                    self.regions.append(rx)
                    # print rx
                    region_color += 1
        return region_color - start_color

    def get_result_map(self):
        return self.result_field

    def get_result_regions(self):
        return self.regions

    def cubify_regions(self):
        """ Create a map where each region will be represented with a bounding box (using min/max x/y values) """
        # init empty map
        self.cubified_field = [[self.SEGMENT_MAP_NOT_COLORED for x in xrange(len(self.field))] for x in
                                xrange(len(self.field[0]))]
        return self.cubify_regions_w_field(self.cubified_field, self.regions)

    def cubify_regions_w_field(self, cubified_field, regions):
        for r in regions:
            for x in range(r.min_x, r.max_x+1):
                for y in range(r.min_y, r.max_y+1):
                    cubified_field[x][y] = r.id
        return cubified_field

    def split_and_cubify_regions(self):
        """ Create a map where each region will be splitted into small rectangles. Afterwards, each region get's a boundingbox"""
        # init empty map
        new_regions = {}
        for r in self.regions:
            # Sort the corresponding cells to scan linewise
            cells = sorted(r.cell_coords)

            # Remember the last row/col while you iterate
            last_row_idx = 0
            last_col_idx = None
            first_line_entry = True
            list_of_last_line_fragments = [RegionLineFragment()]
            line_fragments_in_this_row = 0
            print "calculate with:"
            print cells

            for i in xrange(len(cells)):
                [x,y] = cells[i]
                # Remember the row id, if it's the first elem in the line
                if first_line_entry is True:
                    print "New line at " + str(x) + " " + str(y)
                    last_row_idx = x
                    last_col_idx = y
                    first_line_entry = False
                    line_fragment = list_of_last_line_fragments[-1]
                    line_fragment.min_y = y
                    line_fragment.max_y = y
                    line_fragment.x = x
                    line_fragments_in_this_row = 1
                    # Prepare the hash with the new regions for the new line
                    new_regions[str(x)] = []
                    new_regions[str(x)].append(line_fragment)


                if x is not last_row_idx:
                    print "Found linebreak at " + str(x) + " " + str(y) + " " +str(last_col_idx) + ". fragments in this row: " + str(line_fragments_in_this_row)
                    r = RegionLineFragment()
                    list_of_last_line_fragments.append(r)
                    new_regions[str(x)] = []
                    new_regions[str(x)].append(r)
                    # new_regions[str(last_col_idx)].append(list_of_last_line_fragments[-1])
                    # first_line_entry = True
                    # line_fragment = list_of_last_line_fragments[-1]
                    # line_fragment.min_y = y

                    last_row_idx = x
                    last_col_idx = y
                    first_line_entry = False
                    line_fragment = list_of_last_line_fragments[-1]
                    line_fragment.min_y = y
                    line_fragment.max_y = y
                    line_fragment.x = x
                    line_fragment.cell_coords.append([x,y])
                    # Line break
                    # Maybe we have to merge the last line with it's predecessor
                elif (y-last_col_idx) > self.REGION_SPLIT_SPACE_TRESHOLD+1:
                    print "Split found at " + str(x) + " " + str(y) + " " +str(last_col_idx)

                    r = RegionLineFragment()
                    list_of_last_line_fragments.append(r)
                    new_regions[str(x)].append(r)
                    last_row_idx = x
                    last_col_idx = y
                    first_line_entry = False
                    line_fragment = list_of_last_line_fragments[-1]
                    line_fragment.min_y = y
                    line_fragment.max_y = y
                    line_fragment.x = x
                    line_fragments_in_this_row += 1
                    line_fragment.cell_coords.append([x,y])

                else:
                    line_fragment = list_of_last_line_fragments[-1]
                    line_fragment.max_y = y
                    line_fragment.x = x
                    line_fragment.cell_coords.append([x,y])
                    last_col_idx = y
                print "end of for" + str(last_col_idx)

            # TODO handle the last line for merge

            #print list_of_last_line_fragments
            # created_regions[str(x)] = []
            # created_regions[str(x)].append(list_of_last_line_fragments[-1].to_region())
        first_row_idx = list_of_last_line_fragments[0].x #Start at the lowest x value
        last_row_idx = list_of_last_line_fragments[0].x #Start at the lowest x value
        splitted_regions = {}
        splitted_regions[str(first_row_idx)] = []
        start_color = 100
        print "Dict before iteration"
        for rl in splitted_regions:
            print rl + " value: " + str(splitted_regions[rl])
        for lf in list_of_last_line_fragments:
            r = lf.to_region(start_color)
            # Are we still in the first line? Create a region for each of the line segments
            if lf.x == first_row_idx:
                print "first row - added " + str(r)
                splitted_regions[str(first_row_idx)].append(r)
            else:
                # Only append if merge
                # look into the previous line for regions
                previous_regions = splitted_regions[str(lf.x-1)]
                print "Looking at " + str(lf) + " which is " + str(r)
                for pr in previous_regions:
                    # Init an empty region list first, if necessary
                    if str(lf.x) not in splitted_regions or not splitted_regions[str(lf.x)]:
                        splitted_regions[str(lf.x)] = []

                    print "PR vs. R: " + str(pr) + "----" + str(r)
                    if pr.merge_region_w_criteria(1,1,r):
                        print "true - added " + str(pr)
                        splitted_regions[str(lf.x)].append(pr)
                    else:
                        print "False - added " + str(r)
                        splitted_regions[str(lf.x)].append(r)

            if lf.x is not last_row_idx:
                print "Linebreak in lf iter at "
                last_row_idx = lf.x
            print lf
            start_color += 1
            # print "List of new regions"
            # for rl in splitted_regions:
            #     print rl + " value: " + str(splitted_regions[rl])

        for rl in splitted_regions:
            rl_regions = splitted_regions[rl]
            print rl
            for rlr in rl_regions:
                print rlr
            print "---"




        #     # Scan each line
        #     for x in range(r.min_x, r.max_x+1):
        #         for y in range(r.min_y, r.max_y+1):
        #             self.cubified_field[x][y] = r.id
        # return self.cubified_field


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

    def get_surrounding_cells8(self, x_index, y_index):
        cells = []
        xm1 = False
        xp1 = False
        ym1 = False
        yp1 = False
        if not x_index - 1 < 0:
            cells.append((x_index-1, y_index))
            xm1 = True

        if not y_index - 1 < 0:
            cells.append((x_index, y_index-1))
            ym1 = True

        if not x_index + 1 >= self.map_width:
            cells.append((x_index+1, y_index))
            xp1 = True

        if not y_index + 1 >= self.map_width:
            cells.append((x_index, y_index+1))
            yp1 = True

        if xm1 and ym1:
            cells.append((x_index-1, y_index-1))

        if xm1 and yp1:
            cells.append((x_index-1, y_index+1))

        if xp1 and ym1:
            cells.append((x_index+1, y_index-1))

        if xp1 and yp1:
            cells.append((x_index+1, y_index+1))

        return cells