#!/usr/bin/env python
import rospy
from suturo_planning_interface import scan_map
from suturo_planning_interface.grasp_object import GraspObject
from suturo_planning_interface.place_object import PlaceObject

def main():
  rospy.init_node('suturo_perception_interface', log_level=rospy.DEBUG)
  grasp_object_handler = GraspObject()
  place_object_handler = PlaceObject()
  scan_map.MapScanner()
  rospy.spin()

if __name__ == '__main__':
  main()
