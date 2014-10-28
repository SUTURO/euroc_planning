from math import pi
from geometry_msgs.msg import Quaternion
import rospy
from suturo_planning_manipulation.force_test import plan_to
from suturo_planning_manipulation.manipulation import Manipulation, geometry_msgs
from suturo_planning_manipulation.mathemagie import euler_to_quaternion

__author__ = 'benny'

if __name__ == '__main__':
    rospy.init_node('force_test', anonymous=True)
    m = Manipulation()
    t_point = geometry_msgs.msg.Pose()
    t_point.position.x = -0.4
    t_point.position.y = 0
    t_point.position.z = 0.3
    t_point.orientation = euler_to_quaternion(0, pi, 0)
    m.direct_move(plan_to(t_point, m))
    m.open_gripper()