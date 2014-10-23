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
        self.__value = geometry_msgs.msg.Wrench()
        self.__pub = rospy.Publisher('suturo/torque_force_service', geometry_msgs.msg.Wrench)

    def get_values(self):
        resp = self.__service()
        if not resp.error_message:
            self.__value = resp.external_force
            self.__pub.publish(self.__value)
            return self.__value
        else:
            print resp.error_message

    def is_free(self):
        self.get_values()
        force = abs(self.__value.torque.x) + abs(self.__value.torque.y) + abs(self.__value.torque.z)
        if force > 3:
            return False
        return True