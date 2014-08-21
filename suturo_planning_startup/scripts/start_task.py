#!/usr/bin/env python
import os
import subprocess
import signal
import atexit
import time
import sys
import rospy
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_plans.toplevel import toplevel_plan


_pro_task_selector = None

_save_log = False


def main(task, with_plan, init_sim):

    if init_sim:
        #Taskselector
        print "Starting task_selector"
        global pro_task_selector
        pro_task_selector = subprocess.Popen('rosrun euroc_launch TaskSelector', stdout=subprocess.PIPE,
                                             shell=True, preexec_fn=os.setsid)
    time.sleep(5)

    #If plans should be started start the state machine
    if with_plan:
        rospy.init_node('suturo_planning_execution', log_level=rospy.INFO)
        print 'Started plan'
        toplevel_plan(init_sim)
    else:
        rospy.init_node('suturo_planning_start_task', log_level=rospy.INFO)
        #Start tasks
        start_task(task)

        print 'Waiting for ctrl-c'
        while True:
            time.sleep(0.2)


def exit_handler():
    global _save_log
    if _save_log:
        save_task()
    stop_task()
    time.sleep(2)
    global _pro_task_selector
    if _pro_task_selector:
        os.killpg(_pro_task_selector.pid, signal.SIGTERM)


atexit.register(exit_handler)


if __name__ == '__main__':
    if '--save' in sys.argv:
        save_log = True
    main(sys.argv[1], '--plan' in sys.argv, '--init' in sys.argv)