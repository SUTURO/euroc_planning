#!/usr/bin/env python
import roslib
roslib.load_manifest('suturo_planning_plans')
import suturo_planning_plans


def main():
    suturo_planning_plans.toplevel.toplevel_plan()

if __name__ == '__main__':
    main()