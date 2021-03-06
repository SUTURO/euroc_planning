from copy import deepcopy
from suturo_planning_search.cluster_map import ClusterRegions, Region, RegionLineFragment, RegionType
from suturo_planning_search.cell import Cell

__author__ = 'pmania'

import random
import unittest

class TestClusterRegionsSplitAndCubify(unittest.TestCase):

    def make_free_map(self):
        c = Cell()
        c.set_free()
        test_field = [[deepcopy(c) for x in xrange(15)] for y in xrange(15)]
        return test_field

    def test1(self):
        """ Split the following form into regions:
           xxx
           x x
           x x
        """
        test_field3 = self.make_free_map()

        test_field3[0][6].set_unknown()
        test_field3[0][7].set_unknown()
        test_field3[0][8].set_unknown()

        test_field3[1][6].set_unknown()
        test_field3[2][6].set_unknown()

        test_field3[1][8].set_unknown()
        test_field3[2][8].set_unknown()
        cm = ClusterRegions()
        cm.set_field(test_field3)
        cm.print_field()

        print "Returned regions: " + str(cm.group_regions())
        print "Grouping done"
        cm.print_segmented_field()
        c = cm.get_result_map()

        regions = cm.get_result_regions()
        print "Region count: " + str(len(regions))
        for r in regions:
            print r
            print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
        cubified_field = cm.cubify_regions()
        cm.print_2d_field(cubified_field)
        regions_not_sorted = cm.split_and_cubify_regions()

        # Check the cells of the splitted regions
        regions = sorted(regions_not_sorted)
        self.assertEquals(len(regions), 3)

        muh = [[[0,6],[0,7],[0,8]], [[1,6],[2,6]], [[1,8],[2,8]]]
        self.assertTrue(regions[0].cell_coords in muh)
        self.assertTrue(regions[1].cell_coords in muh)
        self.assertTrue(regions[2].cell_coords in muh)
        # self.assertEquals(regions[2].cell_coords, [[0,6],[0,7],[0,8]])
        # self.assertEquals(regions[1].cell_coords, [[1,6],[2,6]])
        # self.assertEquals(regions[0].cell_coords, [[1,8],[2,8]])

    def test2(self):
        """ Split the following form into one region:
           xxx
           xxx
        """
        print "test2"
        test_field3 = self.make_free_map()

        test_field3[0][6].set_unknown()
        test_field3[0][7].set_unknown()
        test_field3[0][8].set_unknown()

        test_field3[1][6].set_unknown()
        test_field3[1][7].set_unknown()
        test_field3[1][8].set_unknown()
        cm = ClusterRegions()
        cm.set_field(test_field3)
        cm.print_field()

        cm.group_regions()
        # cm.print_segmented_field()
        c = cm.get_result_map()

        # regions = cm.get_result_regions()
        # print "Region count: " + str(len(regions))
        # for r in regions:
        #     print r
        #     print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
        # cubified_field = cm.cubify_regions()
        # cm.print_2d_field(cubified_field)
        regions_not_sorted = cm.split_and_cubify_regions()

        # Check the cells of the splitted regions
        regions = sorted(regions_not_sorted)
        self.assertEquals(len(regions), 1)
        self.assertEquals(regions[0].cell_coords, [[0,6],[0,7],[0,8],[1,6],[1,7],[1,8]])

    def test3(self):
        """ Split the following form into regions:
             x
             x
           xxxxx
        """
        print "test3"
        test_field3 = self.make_free_map()


        test_field3[0][7].set_unknown()
        test_field3[1][7].set_unknown()

        test_field3[2][5].set_unknown()
        test_field3[2][6].set_unknown()
        test_field3[2][7].set_unknown()
        test_field3[2][8].set_unknown()
        test_field3[2][9].set_unknown()

        cm = ClusterRegions()
        cm.set_field(test_field3)
        cm.print_field()

        cm.group_regions()
        # cm.print_segmented_field()
        c = cm.get_result_map()

        # regions = cm.get_result_regions()
        # print "Region count: " + str(len(regions))
        # for r in regions:
        #     print r
        #     print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
        # cubified_field = cm.cubify_regions()
        # cm.print_2d_field(cubified_field)
        regions = cm.split_and_cubify_regions()

        # Check the cells of the splitted regions
        # for rl in regions:
        #     print rl
        self.assertEquals(len(regions), 2)

        self.assertEquals(regions[1].cell_coords, [[0,7],[1,7]])
        self.assertEquals(regions[0].cell_coords, [[2,5],[2,6],[2,7],[2,8],[2,9]])

    def test4(self):
        """ Split the following form into regions:
            xxxx
            x
            xxxx
        """
        print "test4"
        test_field3 = self.make_free_map()

        test_field3[0][5].set_unknown()
        test_field3[0][6].set_unknown()
        test_field3[0][7].set_unknown()
        test_field3[0][8].set_unknown()
        test_field3[1][5].set_unknown()
        test_field3[2][5].set_unknown()
        test_field3[2][6].set_unknown()
        test_field3[2][7].set_unknown()
        test_field3[2][8].set_unknown()


        cm = ClusterRegions()
        cm.set_field(test_field3)
        cm.print_field()

        cm.group_regions()
        # cm.print_segmented_field()
        c = cm.get_result_map()

        # regions = cm.get_result_regions()
        # print "Region count: " + str(len(regions))
        # for r in regions:
        #     print r
        #     print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
        # cubified_field = cm.cubify_regions()
        # cm.print_2d_field(cubified_field)
        regions_not_sorted = cm.split_and_cubify_regions()

        # Check the cells of the splitted regions
        regions = sorted(regions_not_sorted)
        self.assertEquals(len(regions), 3)
        # self.assertEquals(regions[0].cell_coords, [[0,7],[1,7]])
        # self.assertEquals(regions[1].cell_coords, [[2,5],[2,6],[2,7],[2,8],[2,9]])

    def test5(self):
        """ Split the following form into regions:
            xxxx
            x
            x
            xxxx
        """
        print "test5"
        test_field3 = self.make_free_map()

        test_field3[0][5].set_unknown()
        test_field3[0][6].set_unknown()
        test_field3[0][7].set_unknown()
        test_field3[0][8].set_unknown()
        test_field3[1][5].set_unknown()
        test_field3[2][5].set_unknown()
        test_field3[3][5].set_unknown()
        test_field3[3][6].set_unknown()
        test_field3[3][7].set_unknown()
        test_field3[3][8].set_unknown()


        cm = ClusterRegions(True)
        cm.set_field(test_field3)
        cm.print_field()

        cm.group_regions()
        # cm.print_segmented_field()
        c = cm.get_result_map()

        # regions = cm.get_result_regions()
        # print "Region count: " + str(len(regions))
        # for r in regions:
        #     print r
        #     print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
        # cubified_field = cm.cubify_regions()
        # cm.print_2d_field(cubified_field)
        regions_not_sorted = cm.split_and_cubify_regions()

        # Check the cells of the splitted regions
        regions = regions_not_sorted
        self.assertEquals(len(regions), 3)
        self.assertEquals(regions[0].cell_coords, [[3,5],[3,6],[3,7],[3,8]])
        self.assertEquals(regions[1].cell_coords, [[0,5],[0,6],[0,7],[0,8]])
        self.assertEquals(regions[2].cell_coords, [[1,5],[2,5]])

        # self.assertEquals(regions[1].cell_coords, [[2,5],[2,6],[2,7],[2,8],[2,9]])

    def test6(self):
        test_field = self.make_free_map()

        test_field[1][2].set_obstacle()
        test_field[1][2].set_blue()
        test_field[1][3].set_obstacle()
        test_field[1][3].set_blue()
        test_field[1][4].set_obstacle()
        test_field[1][4].set_blue()
        test_field[1][5].set_obstacle()
        test_field[1][5].set_blue()
        test_field[1][6].set_obstacle()
        test_field[1][6].set_green()

        test_field[2][2].set_obstacle()
        test_field[2][2].set_blue()
        test_field[2][3].set_obstacle()
        test_field[2][3].set_green()
        test_field[2][4].set_obstacle()
        test_field[2][4].set_green()
        test_field[2][5].set_obstacle()
        test_field[2][5].set_green()
        test_field[2][6].set_obstacle()
        test_field[2][6].set_green()

        cm = ClusterRegions(True)
        cm.set_field(test_field)
        cm.print_field()

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

        regions = cm.get_result_regions()

        for r in regions:
            print r.get_color_id()

        muh = [[[1, 2], [2, 2], [1, 3], [1, 4], [1, 5]], [[1, 6], [2, 6], [2, 5], [2, 4], [2, 3]]]

        self.assertTrue(regions[0].cell_coords in muh)
        self.assertTrue(regions[1].cell_coords in muh)

class TestRegion(unittest.TestCase):
    def test_line_fragment_to_region(self):
        lf = RegionLineFragment()
        lf.cells = [Cell(), Cell()]
        lf.cell_coords = [[0,0],[0,1]]
        lf.min_y = 0
        lf.max_y = 1
        lf.x = 0
        r = lf.to_region(2)
        self.assertTrue(len(r.cells) == 2)
        self.assertTrue(len(r.cell_coords) == 2)

        self.assertTrue([0,0] in r.cell_coords)
        self.assertTrue([0,1] in r.cell_coords)
        self.assertEquals(r.id,2)

class TestRegionMerge(unittest.TestCase):

    # def setUp(self):
    #     self.seq = range(10)
    #
    # def test_shuffle(self):
    #     # make sure the shuffled sequence does not lose any elements
    #     random.shuffle(self.seq)
    #     self.seq.sort()
    #     self.assertEqual(self.seq, range(10))
    #
    #     # should raise an exception for an immutable sequence
    #     self.assertRaises(TypeError, random.shuffle, (1,2,3))

    def test_region_merge(self):
        r = Region(1)
        r2 = Region(2)
        r_c1 = Cell()
        r_c1.set_free()
        r.cells.append(r_c1)
        r.cell_coords.append([0,0])
        r.min_x = 0
        r.min_y = 0
        r.max_x = 0
        r.max_y = 0


        r2_c1 = Cell()
        r2_c1.set_unknown()
        r2.cells.append(r2_c1)
        r2.cell_coords.append([1,1])
        r2.min_x = 0
        r2.min_y = 0
        r2.max_x = 1
        r2.max_y = 1

        r.merge_region(r2)

        # Check the size of cell lists
        self.assertTrue(len(r.cells) == 2)
        self.assertTrue(len(r.cell_coords) == 2)

        # Check the actual cells
        self.assertTrue(r2_c1 in r.cells)
        self.assertTrue(r_c1 in r.cells)

        self.assertTrue([0,0] in r.cell_coords)
        self.assertTrue([1,1] in r.cell_coords)

        self.assertEqual(r.min_x,0)
        self.assertEqual(r.min_y,0)
        self.assertEqual(r.max_x,1)
        self.assertEqual(r.max_y,1)

    def test_conditional_region_merge(self):
        """ Two blocks below each other. Merge should work
        xx
        xx
        xx
        xx
        """
        r = Region(1)
        r2 = Region(2)

        r.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([0,0])
        r.cell_coords.append([0,1])
        r.cell_coords.append([1,0])
        r.cell_coords.append([1,1])
        r.min_x = 0
        r.min_y = 0
        r.max_x = 1
        r.max_y = 1


        r2.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([2,0])
        r.cell_coords.append([2,1])
        r.cell_coords.append([3,0])
        r.cell_coords.append([3,1])
        r2.min_x = 2
        r2.min_y = 0
        r2.max_x = 3
        r2.max_y = 1

        self.assertTrue(r.merge_region_w_criteria(1,1,r2))

        # Check the size of cell lists
        self.assertTrue(len(r.cells) == 8)
        self.assertTrue(len(r.cell_coords) == 8)

        self.assertTrue([0,0] in r.cell_coords)
        self.assertTrue([0,1] in r.cell_coords)
        self.assertTrue([1,0] in r.cell_coords)
        self.assertTrue([1,1] in r.cell_coords)
        self.assertTrue([2,0] in r.cell_coords)
        self.assertTrue([2,1] in r.cell_coords)
        self.assertTrue([3,0] in r.cell_coords)
        self.assertTrue([3,1] in r.cell_coords)

        self.assertEqual(r.min_x,0)
        self.assertEqual(r.min_y,0)
        self.assertEqual(r.max_x,3)
        self.assertEqual(r.max_y,1)
    def test_conditional_region_merge_2(self):
        """ Two blocks besides and below each other. Shouldn't be merged.
            xx
            xx
              xx
              xx
        """
        r = Region(1)
        r2 = Region(2)

        r.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([0,0])
        r.cell_coords.append([0,1])
        r.cell_coords.append([1,0])
        r.cell_coords.append([1,1])
        r.min_x = 0
        r.min_y = 0
        r.max_x = 1
        r.max_y = 1


        r2.cells = [Cell() for x in xrange(4)]
        r2.cell_coords.append([2,2])
        r2.cell_coords.append([2,3])
        r2.cell_coords.append([3,2])
        r2.cell_coords.append([3,3])
        r2.min_x = 2
        r2.min_y = 2
        r2.max_x = 3
        r2.max_y = 3

        self.assertFalse(r.merge_region_w_criteria(1,1,r2))

        # Check the size of cell lists
        self.assertTrue(len(r.cells) == 4)
        self.assertTrue(len(r.cell_coords) == 4)

    def test_conditional_region_merge3(self):
        """ Two blocks diagonal each other. Merge should work
        xx
         xx
        """
        r = Region(1)
        r2 = Region(2)

        r.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([0,0])
        r.cell_coords.append([0,1])
        r.cell_coords.append([1,0])
        r.cell_coords.append([1,1])
        r.min_x = 0
        r.min_y = 0
        r.max_x = 1
        r.max_y = 1


        r2.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([2,1])
        r.cell_coords.append([2,2])
        r.cell_coords.append([3,1])
        r.cell_coords.append([3,2])
        r2.min_x = 2
        r2.min_y = 1
        r2.max_x = 3
        r2.max_y = 2

        self.assertTrue(r.merge_region_w_criteria(1,1,r2))

        # Check the size of cell lists
        self.assertTrue(len(r.cells) == 8)
        self.assertTrue(len(r.cell_coords) == 8)

        self.assertTrue([0,0] in r.cell_coords)
        self.assertTrue([0,1] in r.cell_coords)
        self.assertTrue([1,0] in r.cell_coords)
        self.assertTrue([1,1] in r.cell_coords)
        self.assertTrue([2,1] in r.cell_coords)
        self.assertTrue([2,2] in r.cell_coords)
        self.assertTrue([3,1] in r.cell_coords)
        self.assertTrue([3,2] in r.cell_coords)

        self.assertEqual(r.min_x,0)
        self.assertEqual(r.min_y,0)
        self.assertEqual(r.max_x,3)
        self.assertEqual(r.max_y,2)

    def test_conditional_region_merge4(self):
        """ Two blocks diagonal of each other. But this time, no left/right tolerance in merge is given.
         Merge should not work
        xx
         xx
        """
        r = Region(1)
        r2 = Region(2)

        r.cells = [Cell() for x in xrange(4)]
        r.cell_coords.append([0,0])
        r.cell_coords.append([0,1])
        r.cell_coords.append([1,0])
        r.cell_coords.append([1,1])
        r.min_x = 0
        r.min_y = 0
        r.max_x = 1
        r.max_y = 1


        r2.cells = [Cell() for x in xrange(4)]
        r2.cell_coords.append([2,1])
        r2.cell_coords.append([2,2])
        r2.cell_coords.append([3,1])
        r2.cell_coords.append([3,2])
        r2.min_x = 2
        r2.min_y = 1
        r2.max_x = 3
        r2.max_y = 2

        self.assertFalse(r.merge_region_w_criteria(0,0,r2))

        # Check the size of cell lists
        self.assertEquals(len(r.cells),4)
        self.assertEquals(len(r.cell_coords),4)

        self.assertTrue([0,0] in r.cell_coords)
        self.assertTrue([0,1] in r.cell_coords)
        self.assertTrue([1,0] in r.cell_coords)
        self.assertTrue([1,1] in r.cell_coords)



if __name__ == '__main__':
    unittest.main()