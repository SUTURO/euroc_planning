import random
import unittest
import rospy
from suturo_planning_search.cell import Cell
from suturo_planning_search.map import Map

class TestCell(unittest.TestCase):
    def test1(self):
        c = Cell()
        self.assertTrue(c.is_unknown())
        self.assertFalse(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertFalse(c.is_free())

        c.set_obstacle()
        self.assertFalse(c.is_unknown())
        self.assertFalse(c.is_object())
        self.assertTrue(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertFalse(c.is_free())

        c.set_free()
        self.assertFalse(c.is_unknown())
        self.assertFalse(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertTrue(c.is_free())

        c.set_object()
        self.assertFalse(c.is_unknown())
        self.assertTrue(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertFalse(c.is_free())

        c.set_mark()
        self.assertFalse(c.is_unknown())
        self.assertTrue(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertTrue(c.is_marked())
        self.assertFalse(c.is_free())

class TestMapCleanUp(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestMapCleanUp, self).__init__(*args, **kwargs)
        # self.gen_stubs()
        rospy.init_node('test_map', anonymous=True)


    def make_free_map(self):
        m = Map(2)
        for x in xrange(m.num_of_cells):
            for y in xrange(m.num_of_cells):
                m.get_cell_by_index(x,y).set_free()
        return m

    def test1_1(self):
        '''
        unknown surrounded by 4 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(4,4).set_unknown()
        m.get_cell_by_index(4,3).set_obstacle()
        m.get_cell_by_index(4,5).set_obstacle()
        m.get_cell_by_index(5,4).set_obstacle()
        m.get_cell_by_index(3,4).set_obstacle()

        self.assertTrue(m.is_cell_surrounded_by_obstacles(4,4))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(4,4).is_obstacle())


    def test1_2(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(4,4).set_unknown()
        m.get_cell_by_index(4,3).set_obstacle()
        m.get_cell_by_index(4,5).set_obstacle()
        m.get_cell_by_index(5,4).set_obstacle()

        self.assertTrue(m.is_cell_surrounded_by_obstacles(4,4))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(4,4).is_obstacle())

    def test1_3(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(0,0).set_unknown()
        m.get_cell_by_index(0,1).set_obstacle()

        self.assertTrue(m.is_cell_surrounded_by_obstacles(0,0))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(0,1).is_obstacle())

    def test1_4(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(5,5).set_unknown()
        m.get_cell_by_index(4,5).set_obstacle()

        self.assertFalse(m.is_cell_surrounded_by_obstacles(5,5))
        self.assertFalse(m.is_cell_alone(5,5))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(5,5).is_unknown())

    def test1_5(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(4,4).set_unknown()
        m.get_cell_by_index(4,3).set_obstacle()
        m.get_cell_by_index(4,5).set_obstacle()

        self.assertFalse(m.is_cell_surrounded_by_obstacles(4,4))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(4,4).is_unknown())

    def test2_1(self):
        '''
        3 single unknowns
        '''
        m = self.make_free_map()

        m.get_cell_by_index(0,0).set_unknown()
        m.get_cell_by_index(0,5).set_unknown()
        m.get_cell_by_index(4,5).set_unknown()

        self.assertTrue(m.get_cell_by_index(0,1).is_free())
        self.assertTrue(m.is_cell_alone(0,0))
        self.assertTrue(m.is_cell_alone(0,5))
        self.assertTrue(m.is_cell_alone(4,5))


        m.clean_up_map()

        self.assertTrue(m.get_cell_by_index(0,0).is_free())
        self.assertTrue(m.get_cell_by_index(0,5).is_free())
        self.assertTrue(m.get_cell_by_index(4,5).is_free())

    def test2_2(self):
        '''
        3 single unknowns
        '''
        m = self.make_free_map()

        m.get_cell_by_index(3,5).set_unknown()
        m.get_cell_by_index(4,5).set_unknown()

        self.assertFalse(m.is_cell_alone(3,5))
        self.assertFalse(m.is_cell_alone(4,5))


        m.clean_up_map()

        self.assertTrue(m.get_cell_by_index(3,5).is_unknown())
        self.assertTrue(m.get_cell_by_index(4,5).is_unknown())


    def test3_1(self):
        '''
        3 single unknowns
        '''
        m = self.make_free_map()

        m.get_cell_by_index(3,5).set_unknown()
        m.get_cell_by_index(4,5).set_obstacle()
        m.get_cell_by_index(4,5).highest_z = 5
        m.get_cell_by_index(3,4).set_obstacle()
        m.get_cell_by_index(3,4).highest_z = 4

        self.assertTrue(m.get_average_z_of_surrounded_obstacles(3,5) == 4.5, m.get_average_z_of_surrounded_obstacles(3,5))

        self.assertTrue(m.get_average_z_of_surrounded_obstacles(0,0) == 1, m.get_average_z_of_surrounded_obstacles(0,0))

    def test4_1(self):
        '''
        3 single unknowns
        '''
        m = self.make_free_map()

        m.get_cell_by_index(3,5).set_obstacle()
        m.get_cell_by_index(4,5).set_obstacle()
        m.get_cell_by_index(3,4).set_obstacle()

        m.mark_region_as_object_under_point(*m.index_to_coordinates(4,5))

        self.assertTrue(m.get_cell_by_index(3,5).is_object())
        self.assertTrue(m.get_cell_by_index(4,5).is_object())
        self.assertTrue(m.get_cell_by_index(3,4).is_object())

    def test5_1(self):
        '''
        3 single unknowns
        '''
        m = Map(2)
        self.assertTrue(m.get_percent_cleared() == 0.0, m.get_percent_cleared())

        m = self.make_free_map()
        self.assertTrue(m.get_percent_cleared() == 1.0, m.get_percent_cleared())

        m.get_cell_by_index(3,4).set_unknown()
        self.assertTrue(m.get_percent_cleared() < 1.0, m.get_percent_cleared())

    def test6_1(self):
        '''
        '''

        m = self.make_free_map()
        # print m.coordinates_to_index(0.01, 0.01)
        # print m.coordinates_to_index(0.13, 0.09)
        # print "---"
        # cells = m.get_cells_between(0.01, 0.01, 0.13, 0.09)
        # print len(cells)
        # print m.cell_size_x
        cells = m.get_cells_between(0.01, 0.01, 0.13, 0.09)
        self.assertTrue(len(cells) == 5)
        self.assertTrue(m.coordinates_to_index(0.01, 0.01) in cells)
        self.assertTrue(m.coordinates_to_index(0.13, 0.09) in cells)

        cells = m.get_cells_between(0.13, 0.09, 0.01, 0.01)
        self.assertTrue(len(cells) == 5)
        self.assertTrue(m.coordinates_to_index(0.01, 0.01) in cells)
        self.assertTrue(m.coordinates_to_index(0.13, 0.09) in cells)

        cells = m.get_cells_between(0.01, 0.01, 0.09, 0.09)
        self.assertTrue(len(cells) == 3)
        self.assertTrue(m.coordinates_to_index(0.01, 0.01) in cells)
        self.assertTrue(m.coordinates_to_index(0.09, 0.09) in cells)
        #
        # m.get_cell_by_index(3,4).set_unknown()
        # self.assertTrue(m.get_percent_cleared() < 1.0, m.get_percent_cleared())













