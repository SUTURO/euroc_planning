#!/usr/bin/env python
import os
import subprocess
import atexit
import time
from suturo_planning_task_selector import start_task, stop_task, save_task

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
    save_task()
    stop_task()

    start_task('task2_v1_3')
    time.sleep(10)
    save_task()
    stop_task()

    start_task('task3_v1')
    time.sleep(10)
    save_task()
    stop_task()

    start_task('task4_v1_3')
    time.sleep(10)
    save_task()
    stop_task()

    start_task('task5_v1')
    time.sleep(10)
    save_task()
    stop_task()

    start_task('task6_v1')
    time.sleep(10)
    save_task()
    stop_task()


def exit_handler():
    time.sleep(2)
    global pro_task_selector
    assert isinstance(pro_task_selector, subprocess.Popen)
    pro_task_selector.kill()


atexit.register(exit_handler)


if __name__ == '__main__':
    start_demo()