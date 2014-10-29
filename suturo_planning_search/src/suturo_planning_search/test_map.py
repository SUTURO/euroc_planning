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
        self.assertFalse(c.is_blue())
        self.assertFalse(c.is_red())
        self.assertTrue(c.is_undef(), c.get_color_id())
        self.assertTrue(c.get_color() == c.UNDEF)

        c.set_blue()
        self.assertFalse(c.is_unknown())
        self.assertFalse(c.is_object())
        self.assertTrue(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertFalse(c.is_free())
        self.assertTrue(c.is_blue())
        self.assertFalse(c.is_red())
        self.assertFalse(c.is_undef())

        c.set_free()
        self.assertFalse(c.is_unknown())
        self.assertFalse(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertTrue(c.is_free())
        self.assertFalse(c.is_blue())
        self.assertFalse(c.is_red())

        c.set_object()
        self.assertFalse(c.is_unknown())
        self.assertTrue(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertFalse(c.is_marked())
        self.assertFalse(c.is_free())
        self.assertTrue(c.is_blue())
        self.assertFalse(c.is_red())
        self.assertFalse(c.is_undef())

        c.set_mark()
        self.assertFalse(c.is_unknown())
        self.assertTrue(c.is_object())
        self.assertFalse(c.is_obstacle())
        self.assertTrue(c.is_marked())
        self.assertFalse(c.is_free())
        self.assertTrue(c.is_blue())
        self.assertFalse(c.is_red())

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

        self.assertTrue(m.get_surrounding_obstacles(4,4))

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

        self.assertTrue(len(m.get_surrounding_obstacles(4,4)) == 3)

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(4,4).is_obstacle())

    # def test1_3(self):
    #     '''
    #     unknown surrounded by 3 obstacles
    #     '''
    #     m = self.make_free_map()
    #
    #     m.get_cell_by_index(0,0).set_unknown()
    #     m.get_cell_by_index(0,1).set_obstacle()
    #
    #     self.assertTrue(m.is_cell_surrounded_by_obstacles(0,0))
    #
    #     m.clean_up_map()
    #     self.assertTrue(m.get_cell_by_index(0,1).is_obstacle())

    def test1_4(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(5,5).set_unknown()
        m.get_cell_by_index(4,5).set_obstacle()

        # self.assertFalse(m.is_cell_surrounded_by_obstacles(5,5))
        self.assertFalse(m.is_cell_alone(5,5))

        m.clean_up_map()
        self.assertTrue(m.get_cell_by_index(5,5).is_unknown(), m)

    def test1_5(self):
        '''
        unknown surrounded by 3 obstacles
        '''
        m = self.make_free_map()

        m.get_cell_by_index(4,4).set_unknown()
        m.get_cell_by_index(4,3).set_obstacle()
        m.get_cell_by_index(4,5).set_obstacle()

        # self.assertFalse(m.is_cell_surrounded_by_obstacles(4,4))

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

        # print m
        self.assertTrue(m.get_cell_by_index(0,1).is_free())
        self.assertTrue(m.is_cell_alone(4,5), m)
        self.assertTrue(m.is_cell_alone(0,0), m)
        self.assertTrue(m.is_cell_alone(0,5), m)



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

        self.assertTrue(m.get_cell_by_index(3,5).is_object(), m)
        self.assertTrue(m.get_cell_by_index(4,5).is_object(), m)
        self.assertTrue(m.get_cell_by_index(3,4).is_object(), m)

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

    def test7_1(self):
        m = self.make_free_map()

        m.get_cell_by_index(10,10).set_obstacle()
        m.get_cell_by_index(10,10).highest_z = 0.5
        m.get_cell_by_index(10,11).set_obstacle()
        m.get_cell_by_index(10,11).highest_z = 1
        m.get_cell_by_index(10,12).set_obstacle()
        m.get_cell_by_index(10,12).highest_z = 1
        m.get_cell_by_index(10,13).set_obstacle()
        m.get_cell_by_index(10,13).highest_z = 1
        m.get_cell_by_index(10,14).set_obstacle()
        m.get_cell_by_index(10,14).highest_z = 1
        m.get_cell_by_index(11,10).set_obstacle()
        m.get_cell_by_index(11,10).highest_z = 1
        m.get_cell_by_index(11,11).set_obstacle()
        m.get_cell_by_index(11,11).highest_z = 1
        m.get_cell_by_index(11,12).set_obstacle()
        m.get_cell_by_index(11,12).highest_z = 1
        m.get_cell_by_index(11,13).set_obstacle()
        m.get_cell_by_index(11,13).highest_z = 1

        rs = m.get_obstacle_regions()
        print m.get_cell_volume_by_index(10, 10)
        self.assertTrue(m.get_cell_volume_by_index(10, 10) == 0.0008)
        print m.get_region_volume(rs[0])
        self.assertTrue(abs(m.get_region_volume(rs[0]) - 0.0136) <= 0.00001, m.get_region_volume(rs[0]))

    def test8_1(self):
        m = self.make_free_map()

        m.get_cell_by_index(10,10).set_unknown()
        m.get_cell_by_index(10,11).set_unknown()
        m.get_cell_by_index(10,9).set_unknown()
        m.get_cell_by_index(9,10).set_unknown()
        m.get_cell_by_index(11,10).set_unknown()
        m.get_cell_by_index(11,11).set_unknown()
        m.get_cell_by_index(9,9).set_unknown()
        m.get_cell_by_index(11,9).set_unknown()
        # m.get_cell_by_index(11,13).set_unknown()

        rs = m.get_unknown_regions()
        boarder = m.get_boarder_cells_points(rs[0])
        self.assertTrue(len(boarder) == 7, str(m) + "\n" + str(len(boarder)))

    def test9_1(self):
        '''
        fffffff
        ffuufff
        ffuuuff
        ffuuuff
        fffffff
        '''
        m = self.make_free_map()

        m.get_cell_by_index(10,10).set_unknown()
        m.get_cell_by_index(10,11).set_unknown()
        m.get_cell_by_index(10,9).set_unknown()
        m.get_cell_by_index(9,10).set_unknown()
        m.get_cell_by_index(11,10).set_unknown()
        m.get_cell_by_index(11,11).set_unknown()
        m.get_cell_by_index(9,9).set_unknown()
        m.get_cell_by_index(11,9).set_unknown()

        self.assertTrue(m.is_more_edge_by_index(10,10,10,11) == -1, m.is_more_edge_by_index(10,10,10,11))
        self.assertTrue(m.is_more_edge_by_index(11,11,10,10) == 1, m.is_more_edge_by_index(11,11,10,10))


    def test10_1(self):
        '''
        fffffff
        ffuufff
        ffuuuff
        ffuuuff
        fffffff
        '''
        m = self.make_free_map()

        m.get_cell_by_index(10,10).set_unknown()

        l = m.get_surrounding_cells8_by_index(10, 10)
        self.assertTrue(len(l) == 8, len(l))

        m.get_cell_by_index(9,9).set_unknown()
        self.assertTrue(m.get_cell_by_index(9,9) in map(lambda c: c[0], l))
        m.get_cell_by_index(9,9).set_free()

        m.get_cell_by_index(10,9).set_unknown()
        self.assertTrue(m.get_cell_by_index(10,9) in map(lambda c: c[0], l))
        m.get_cell_by_index(10,9).set_free()

        m.get_cell_by_index(11,9).set_unknown()
        self.assertTrue(m.get_cell_by_index(11,9) in map(lambda c: c[0], l))
        m.get_cell_by_index(11,9).set_free()

        m.get_cell_by_index(9,10).set_unknown()
        self.assertTrue(m.get_cell_by_index(9,10) in map(lambda c: c[0], l))
        m.get_cell_by_index(9,10).set_free()

        m.get_cell_by_index(11,10).set_unknown()
        self.assertTrue(m.get_cell_by_index(11,10) in map(lambda c: c[0], l))
        m.get_cell_by_index(11,10).set_free()

        m.get_cell_by_index(9,11).set_unknown()
        self.assertTrue(m.get_cell_by_index(9,11) in map(lambda c: c[0], l))
        m.get_cell_by_index(9,11).set_free()

        m.get_cell_by_index(10,11).set_unknown()
        self.assertTrue(m.get_cell_by_index(10,10) in map(lambda c: c[0], l))
        m.get_cell_by_index(10,11).set_free()

        m.get_cell_by_index(11,11).set_unknown()
        self.assertTrue(m.get_cell_by_index(11,11) in map(lambda c: c[0], l))
        m.get_cell_by_index(11,11).set_free()











