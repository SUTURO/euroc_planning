from twisted.trial import unittest
# from geometry_msgs.msg._Point import Point
from suturo_planning_manipulation.mathemagie import *


class TestMapCleanUp(unittest.TestCase):
    
    def test1_1(self):
        v1 = Point(1,0,0)
        v2 = Point(0,1,0)

        self.assertTrue(get_angle(v1,v2) == pi/2, get_angle(v1,v2))

    def test1_2(self):
        v1 = Point(1,0,0)
        v2 = Point(1,1,0)

        self.assertTrue(pi/4-0.01 <= get_angle(v1,v2) <= pi/4+0.01, get_angle(v1,v2))

    def test2_1(self):
        v1 = Point(0.5,0,0)
        self.assertTrue(normalize(v1).x == 1)

    def test3_1(self):
        v1 = Point(0.5,0,0)
        self.assertTrue(magnitude(v1) == 0.5)

    def test4_1(self):
        v1 = Point(1,2,3)
        v2 = Point(3, 2, 1)
        self.assertTrue(dot_product(v1, v2) == 10)

    def test5_1(self):
        v1 = Point(1,2,3)
        v2 = Point(3, 2, 1)
        self.assertTrue(subtract_point(v1, v2).x == -2)
        self.assertTrue(subtract_point(v1, v2).y == 0)
        self.assertTrue(subtract_point(v1, v2).z == 2)

    def test6_1(self):
        v1 = Point(1,2,3)
        v2 = Point(3, 2, 1)
        self.assertTrue(add_point(v1, v2).x == 4)
        self.assertTrue(add_point(v1, v2).y == 4)
        self.assertTrue(add_point(v1, v2).z == 4)

    def test7_1(self):
        v1 = Point(1,2,-3)
        s = -7
        self.assertTrue(multiply_point(s, v1).x == -7)
        self.assertTrue(multiply_point(s, v1).y == -14)
        self.assertTrue(multiply_point(s, v1).z == 21)