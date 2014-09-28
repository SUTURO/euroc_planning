#!/usr/bin/env python
import os
import subprocess
import signal
import atexit
import sys
import rospy
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_plans.toplevel import toplevel_plan


_pro_task_selector = None

_save_log = False


def main(task, with_plan, init_sim):

    # signal.signal(signal.SIGINT, exit_handler)

    if init_sim:
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
        toplevel_plan(init_sim, [task])
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
    if not rospy.is_shutdown():
        if _save_log:
            rospy.loginfo('Saving log')
            save_task()
        stop_task()
        rospy.sleep(2)
    else:
        rospy.init_node('exit_handler')
        if _save_log:
            rospy.loginfo('Saving log')
            save_task()
        stop_task()
        rospy.sleep(2)


def exit_handler():
    print 'Stopping TaskSelector'
    global _pro_task_selector
    if _pro_task_selector is not None:
        print 'Stopping gazebo'
        _pro_task_selector.terminate()
        os.killpg(_pro_task_selector.pid, signal.SIGKILL)


rospy.on_shutdown(rospy_exit_handler)


if __name__ == '__main__':
    if '--save' in sys.argv:
        _save_log = True
    main(sys.argv[1], '--plan' in sys.argv, '--init' in sys.argv)