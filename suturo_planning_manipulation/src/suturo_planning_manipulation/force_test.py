#!/usr/bin/env python

__author__ = 'benny'

from manipulation import *
from torque_force_service import TorqueForceService


if __name__ == '__main__':
    rospy.init_node('force_test', anonymous=True)
    # m = Manipulation()
    # m.close_gripper()
    # t_point = geometry_msgs.msg.PoseStamped()
    # t_point.header.frame_id = "/odom_combined"
    # t_point.pose.position = geometry_msgs.msg.Point(0.6, -0.6, 0.5)
    # t_point.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    # m.move_to(t_point)
    tfs = TorqueForceService()