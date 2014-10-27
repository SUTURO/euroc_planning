#!/usr/bin/env python
import os
import getopt
import subprocess
import signal
import atexit
from datetime import datetime
import sys
import rospy
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_plans.toplevel import toplevel_plan
from suturo_planning_task_selector import task_selector


_pro_task_selector = None

_save_log = False


def main(task, with_plan, init_sim, savelog, no_taskselector, initialization_time, log_to_console_only):

    # signal.signal(signal.SIGINT, exit_handler)
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
        toplevel_plan(init_sim, [task], savelog, initialization_time, log_to_console_only)
    else:
        rospy.init_node('suturo_planning_start_task', log_level=rospy.DEBUG)
        #Start tasks
        start_task(task)

        print 'Waiting for ctrl-c'
        rospy.spin()


def rospy_exit_handler():
    atexit.register(exit_handler)
    global _save_log
    print 'Exithandler: save_log = ' + str(_save_log) + ', rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    if _save_log and not task_selector.task_saved:
        rospy.loginfo('Going to save log')
        save_task()
    if not task_selector.task_stopped:
        stop_task()
    rospy.sleep(2)


def exit_handler():
    global _pro_task_selector
    if _pro_task_selector is not None:
        print 'Stopping TaskSelector'
        print 'Stopping gazebo'
        os.killpg(_pro_task_selector.pid, signal.SIGTERM)
        rospy.sleep(0.5)
        os.killpg(_pro_task_selector.pid, signal.SIGKILL)


if '--init' in sys.argv:
    rospy.on_shutdown(rospy_exit_handler)

if __name__ == '__main__':
    task = sys.argv[1]
    argv = sys.argv[2:]
    try:
        opts, args = getopt.getopt(argv, 'c', ['save', 'plan', 'init', 'no-ts', 'inittime='])
    except getopt.GetoptError:
        sys.exit(2)
    #print ('opts: ' + str(opts))
    #print ('args: ' + str(args))
    initialization_time = None
    console_only = False
    for opt, arg in opts:
        #print ('opt: ' + str(opt))
        #print ('arg: ' + str(arg))
        if opt == '--save':
            _save_log = True
        elif opt == '--inittime':
            __initialization_time = arg
        elif opt in ['-c']:
            console_only = True

    if initialization_time is None:
        initialization_time = datetime.now().isoformat(' ')
    main(task, '--plan' in sys.argv, '--init' in sys.argv, _save_log, '--no-ts' in sys.argv, initialization_time,
         console_only)