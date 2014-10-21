from suturo_planning_search.cluster_map import ClusterRegions, Region
from suturo_planning_search.cell import Cell

__author__ = 'pmania'

import random
import unittest

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

        r.merge_region(r2)

        # Check the size of cell lists
        self.assertTrue(len(r.cells) == 8)
        self.assertTrue(len(r.cell_coords) == 8)


if __name__ == '__main__':
    unittest.main()