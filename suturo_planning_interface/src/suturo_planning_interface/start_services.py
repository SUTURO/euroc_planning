#!/usr/bin/env python
import rospy
from suturo_planning_interface import scan_map
import suturo_planning_interface.grasp_object

def main():
  rospy.init_node('suturo_perception_interface', log_level=rospy.DEBUG)
  grasp_object_handler = suturo_planning_interface.grasp_object.GraspObject()
  scan_map.MapScanner()
  rospy.spin()

if __name__ == '__main__':
  main()
