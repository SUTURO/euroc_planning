#!/usr/bin/env python
import suturo_planning_plans
import rospy


def main():
    #rospy.init_node('suturo_planning')
    suturo_planning_plans.toplevel.toplevel_plan()

if __name__ == '__main__':
    main()