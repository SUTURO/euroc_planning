#!/usr/bin/env python
import rospy
from suturo_planning_interface import scan_map

def main():
  rospy.init_node('suturo_perception_interface', log_level=rospy.DEBUG)
  scan_map.MapScanner()
  rospy.spin()

if __name__ == '__main__':
  main()
