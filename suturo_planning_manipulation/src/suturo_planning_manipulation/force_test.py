#!/usr/bin/env python

__author__ = 'benny'

from euroc_c2_msgs.srv import GetEstimatedExternalForce
import rospy
from cam_manipulation import CamManipulation


if __name__ == '__main__':
    c = CamManipulation()
    c.pan(0.5)
