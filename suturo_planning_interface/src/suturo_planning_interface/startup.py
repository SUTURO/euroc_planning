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
from suturo_planning_plans import start_nodes
from suturo_planning_interface.toplevel import Toplevel


_pro_task_selector = None
_save_log = False
__handling_exit = False
toplevel = None


def exit_handler(signum=None, frame=None):
    print('startup: exit_handler')
    print('startup: signum: ' + str(signum) + ', frame: ' + str(frame))
    global __handling_exit
    global _save_log
    print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    if __handling_exit:
        print('startup: Already handling exit.')
        return
    __handling_exit = True
    print('startup: Checking for task selector.')
    global _pro_task_selector
    if _pro_task_selector is not None:
        print 'Stopping TaskSelector'
        print 'Stopping gazebo'
        exterminate(_pro_task_selector.pid, signal.SIGINT)
    #print('Exiting moveit_commander.')
    #moveit_commander.os._exit(0)
    print('startup: Exiting exit_handler')


signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)
atexit.register(exit_handler)


def main(initialization_time, logging):
    start_task_selector()
    rospy.sleep(10)
    global toplevel
    print "Start toplevel"
    toplevel = Toplevel(initialization_time, logging)


def start_task_selector():
    print "Starting task_selector"
    global _pro_task_selector
    _pro_task_selector = subprocess.Popen('rosrun euroc_launch TaskSelector', stdout=subprocess.PIPE,
                                          shell=True, preexec_fn=os.setsid)


def rospy_exit_handler():
    print('startup: rospy_exit_handler')
    global _save_log
    if not start_nodes.executed_test_node_check:
        start_nodes.check_node(initialization_time, logging)
    print 'save_log = ' + str(_save_log) + ', rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    print('Checking save task.')
    if _save_log and not task_selector.task_saved:
        rospy.loginfo('Going to save log')
        save_task()
    print('Checking stop task.')
    if not task_selector.task_stopped:
        rospy.loginfo('Going to stop task.')
        stop_task()
    print('Going to sleep a sec.')
    time.sleep(2)
    print('startup: Exiting rospy_exit_handler.')

if __name__ == '__main__':
    task = sys.argv[1]
    argv = sys.argv[2:]
    try:
        opts, args = getopt.getopt(argv, '', ['save', 'plan', 'init', 'no-ts', 'inittime=', 'logging=', 'parent='])
    except getopt.GetoptError:
        print('startup: Could not parse parameters.')
        sys.exit(2)
    #print ('opts: ' + str(opts))
    #print ('args: ' + str(args))
    initialization_time = None
    logging = 0
    parent = None
    for opt, arg in opts:
        # print ('opt: ' + str(opt))
        # print ('arg: ' + str(arg))
        if opt == '--save':
            _save_log = True
        elif opt == '--init':
            rospy.on_shutdown(rospy_exit_handler)
        elif opt == '--inittime':
            initialization_time = arg
        elif opt in ['--logging']:
            l = int(arg)
            if l in [0, 1, 2]:
                logging = l
        elif opt in ['--parent']:
            parent = int(arg)

    if initialization_time is None:
        initialization_time = datetime.now().isoformat('-')
    print('startup: Going to execute main.')
    main(initialization_time, logging)
