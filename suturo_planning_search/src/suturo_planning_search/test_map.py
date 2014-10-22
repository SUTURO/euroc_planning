import random
import unittest
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

