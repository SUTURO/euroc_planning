#!/usr/bin/env python
import rospy
from suturo_planning_interface import scan_map
from suturo_planning_interface.grasp_object import GraspObject

def main():
  rospy.init_node('suturo_perception_interface', log_level=rospy.DEBUG)
  grasp_object_handler = GraspObject()
  scan_map.MapScanner()
  rospy.spin()

if __name__ == '__main__':
  main()
