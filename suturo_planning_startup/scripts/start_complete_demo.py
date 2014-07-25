#!/usr/bin/env python
import os
import signal
import subprocess
import atexit
import time
import rospy
from euroc_c2_msgs.srv import *


pro_task_selector = 0


def start_demo():

    #Taskselector
    global pro_task_selector
    pro_task_selector = subprocess.Popen('/opt/euroc_c2s1/start_euroc_task_selector', stdout=subprocess.PIPE,
                                         shell=True, preexec_fn=os.setsid)
    time.sleep(5)

    #Start tasks
    start_task('task1_v1')
    time.sleep(10)
    stop_task()

    start_task('task2_v1_3')
    time.sleep(10)
    stop_task()

    start_task('task3_v1')
    time.sleep(10)
    stop_task()

    start_task('task4_v1_3')
    time.sleep(10)
    stop_task()

    start_task('task5_v1')
    time.sleep(10)
    stop_task()

    start_task('task6_v1')
    time.sleep(10)
    stop_task()


def start_task(scene):
    print 'Starting ' + scene
    rospy.wait_for_service('euroc_c2_task_selector/start_simulator')
    try:
        start_simulator = rospy.ServiceProxy('euroc_c2_task_selector/start_simulator', StartSimulator)
        return start_simulator('suturo', scene)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def stop_task():
    save_task()
    print 'Stopping task'
    rospy.wait_for_service('euroc_c2_task_selector/stop_simulator')
    try:
        stop_simulator = rospy.ServiceProxy('euroc_c2_task_selector/stop_simulator', StopSimulator)
        return stop_simulator()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def save_task():
    print 'Saving log'
    rospy.wait_for_service('/euroc_interface_node/save_log')
    try:
        save_log = rospy.ServiceProxy('/euroc_interface_node/save_log', SaveLog)
        return save_log()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def exit_handler():
    time.sleep(2)


atexit.register(exit_handler)


if __name__ == '__main__':
    start_demo()