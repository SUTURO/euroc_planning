import random
import unittest
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from manipulation_msgs.msg import _Grasp
from manipulation_msgs.srv import _GraspPlanning
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from calc_grasp_position import get_fingertip
from mathemagie import euler_to_quaternion
from place import get_grasped_part
from suturo_planning_manipulation.calc_grasp_position import make_scan_pose
from suturo_planning_search.cell import Cell
from suturo_planning_search.map import Map
from math import sqrt, pi, cos, sin, acos



class TestMapCleanUp(unittest.TestCase):

    # def __init__(self, *args, **kwargs):
    #     super(TestMapCleanUp, self).__init__(*args, **kwargs)
    #     # self.gen_stubs()
    #     rospy.init_node('test_map', anonymous=True)

    def test1(self):
        p = PoseStamped()
        p.header.frame_id = "/odom_combined"
        p.pose.position = Point(1,0,0)
        p.pose.orientation = euler_to_quaternion(0,0,0)
        p1 = get_fingertip(p)
        p2 = Point(1.2185, 0,0)
        self.assertTrue(abs(p1.point.x - p2.x) < 0.0001)
        self.assertTrue(abs(p1.point.y - p2.y) < 0.0001)
        self.assertTrue(abs(p1.point.z - p2.z) < 0.0001)

    def test1_1(self):
        p = PoseStamped()
        p.header.frame_id = "/odom_combined"
        p.pose.position = Point(1,0,0)
        p.pose.orientation = euler_to_quaternion(0,pi,0)
        p2 = Point(0.7815,0,0)
        p1 = get_fingertip(p)
        self.assertTrue(abs(p1.point.x - p2.x) < 0.0001)
        self.assertTrue(abs(p1.point.y - p2.y) < 0.0001)
        self.assertTrue(abs(p1.point.z - p2.z) < 0.0001)

    def test1_2(self):
        p = PoseStamped()
        p.header.frame_id = "/odom_combined"
        p.pose.position = Point(1,0,0)
        p.pose.orientation = euler_to_quaternion(0,0,pi/2)
        p2 = Point(1,0.2185,0)
        p1 = get_fingertip(p)
        # print p1
        self.assertTrue(abs(p1.point.x - p2.x) < 0.0001)
        self.assertTrue(abs(p1.point.y - p2.y) < 0.0001)
        self.assertTrue(abs(p1.point.z - p2.z) < 0.0001)

    def test2_1(self):

        co = CollisionObject()
        co.operation = CollisionObject.ADD
        co.id = "muh"
        co.header.frame_id = "/odom_combined"
        cylinder = SolidPrimitive()
        cylinder.type = SolidPrimitive.CYLINDER
        cylinder.dimensions.append(0.3)
        cylinder.dimensions.append(0.03)
        co.primitives = [cylinder]
        co.primitive_poses = [Pose()]
        co.primitive_poses[0].position = Point(1.2185, 0,0)
        co.primitive_poses[0].orientation = Quaternion(0,0,0,1)

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions.append(0.1)
        box.dimensions.append(0.1)
        box.dimensions.append(0.1)
        co.primitives.append(box)
        co.primitive_poses.append(Pose())
        co.primitive_poses[1].position = Point(1.1185, 0,0)
        co.primitive_poses[1].orientation = Quaternion(0,0,0,1)

        co.primitives.append(box)
        co.primitive_poses.append(Pose())
        co.primitive_poses[2].position = Point(0, 0,0)
        co.primitive_poses[2].orientation = Quaternion(0,0,0,1)


        p = PoseStamped()
        p.header.frame_id = "/odom_combined"
        p.pose.position = Point(1,0,0)
        p.pose.orientation = euler_to_quaternion(0,0,0)
        self.assertEqual(get_grasped_part(co, get_fingertip(p))[1], 0)





















