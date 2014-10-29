#!/usr/bin/env python

__author__ = 'benny'

import rospy
from geometry_msgs import msg
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from std_msgs import *


class TorqueForceService(object):
    def __init__(self):
        servicename = '/euroc_interface_node/get_estimated_external_force'
        rospy.wait_for_service(servicename)
        self.__service = rospy.ServiceProxy(servicename, GetEstimatedExternalForce)

    def get_values(self):
        resp = self.__service()
        if not resp.error_message:
            self.__value = resp.external_force
            return self.__value
        else:
            print resp.error_message

    def is_free(self):
        resp = self.__service()
        if not resp.error_message:
            force = abs(resp.external_force.torque.x) + abs(resp.external_force.torque.y) + abs(resp.external_force.torque.z)
            force += abs(resp.external_force.force.x) + abs(resp.external_force.force.y) + abs(resp.external_force.force.z)
            print resp
            if force > 3:
                return False
            return True
        else:
            print resp.error_message