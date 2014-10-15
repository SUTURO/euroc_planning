#!/usr/bin/env python

__author__ = 'benny'

from euroc_c2_msgs.srv import GetEstimatedExternalForce
import rospy
from cam_manipulation import CamManipulation
from manipulation import *


if __name__ == '__main__':
    rospy.init_node('force_test', anonymous=True)
    m = Manipulation()
    # m.close_gripper()
    t_point = geometry_msgs.msg.PoseStamped()
    t_point.header.frame_id = "/odom_combined"
    t_point.pose.position = geometry_msgs.msg.Point(0.6, -0.6, 0.5)
    t_point.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    m.move_to(t_point)