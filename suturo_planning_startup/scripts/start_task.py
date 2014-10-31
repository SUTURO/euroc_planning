#!/usr/bin/env python
import os
import getopt
import subprocess
import signal
import atexit
from datetime import datetime
import sys
import rospy
#import moveit_commander
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_plans.toplevel import toplevel_plan
from suturo_planning_task_selector import task_selector
from suturo_planning_manipulation.total_annihilation import exterminate
from suturo_planning_plans import toplevel
from suturo_planning_plans import start_nodes


_pro_task_selector = None
_save_log = False
__handling_exit = False


def exit_handler(signum=None, frame=None):
    print('start_task: exit_handler')
    global __handling_exit
    global _save_log
    print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    if __handling_exit:
        print('start_task: Already handling exit.')
        return
    __handling_exit = True
    print('start_task: Checking for task selector.')
    global _pro_task_selector
    if _pro_task_selector is not None:
        print 'Stopping TaskSelector'
        print 'Stopping gazebo'
        exterminate(_pro_task_selector.pid, signal.SIGINT)
    #print('Exiting moveit_commander.')
    #moveit_commander.os._exit(0)
    print('start_task: Exiting exit_handler')


signal.signal(signal.SIGINT, exit_handler)
signal.signal(signal.SIGTERM, exit_handler)
atexit.register(exit_handler)


def main(task, with_plan, init_sim, savelog, no_taskselector, initialization_time, logging):
    print('start_task: main')
    if init_sim and not no_taskselector:
        #Taskselector
        print "Starting task_selector"
        global _pro_task_selector
        _pro_task_selector = subprocess.Popen('rosrun euroc_launch TaskSelector', stdout=subprocess.PIPE,
                                              shell=True, preexec_fn=os.setsid)
    rospy.sleep(5)

    #If plans should be started start the state machine
    if with_plan:
        rospy.init_node('suturo_planning_execution', log_level=rospy.DEBUG)
        rospy.loginfo('Started plan')
        toplevel_plan(init_sim, [task], savelog, initialization_time, logging)
    else:
        rospy.init_node('suturo_planning_start_task', log_level=rospy.DEBUG)
        #Start tasks
        start_task(task)

        print 'Waiting for ctrl-c'
        rospy.spin()
        print('start_task: exiting main')


def rospy_exit_handler():
    print('start_task: rospy_exit_handler')
    global _save_log
    if not start_nodes.executed_test_node_check:
        start_nodes.check_node(initialization_time, logging)
    print 'save_log = ' + str(_save_log) + ', rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    if _save_log and not task_selector.task_saved:
        rospy.loginfo('Going to save log')
        save_task()
    if not task_selector.task_stopped:
        stop_task()
    rospy.sleep(2)
    print('start_task: Exiting rospy_exit_handler.')

if __name__ == '__main__':
    task = sys.argv[1]
    argv = sys.argv[2:]
    try:
        opts, args = getopt.getopt(argv, '', ['save', 'plan', 'init', 'no-ts', 'inittime=', 'logging='])
    except getopt.GetoptError:
        sys.exit(2)
    #print ('opts: ' + str(opts))
    #print ('args: ' + str(args))
    initialization_time = None
    logging = 0
    for opt, arg in opts:
        #print ('opt: ' + str(opt))
        #print ('arg: ' + str(arg))
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

    if initialization_time is None:
        initialization_time = datetime.now().isoformat('-')
    main(task, '--plan' in sys.argv, '--init' in sys.argv, _save_log, '--no-ts' in sys.argv, initialization_time,
         logging)