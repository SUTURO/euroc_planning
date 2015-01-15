#!/usr/bin/env python
import os
import getopt
import subprocess
import signal
import atexit
from datetime import datetime
import sys
import rospy
import time
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_task_selector import task_selector
from suturo_planning_manipulation.total_annihilation import exterminate
from suturo_planning_interface import start_nodes
from suturo_planning_interface.toplevel import Toplevel

def main():
    print('Test2')
    rospy.init_node('suturo_toplevel2', log_level=rospy.DEBUG)
    rospy.set_param('use_sim_time', True)
    #start_task("task1_v1")

if __name__ == '__main__':
    print('Test.')
    main()
