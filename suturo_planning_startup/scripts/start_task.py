#!/usr/bin/env python
import os
import subprocess
import atexit
import time
import sys
from suturo_planning_task_selector import start_task, stop_task, save_task


pro_task_selector = 0

save_log = False


def main(task):

    #Taskselector
    print "Starting task_selector"
    global pro_task_selector
    pro_task_selector = subprocess.Popen('/opt/euroc_c2s1/start_euroc_task_selector', stdout=subprocess.PIPE,
                                         shell=True, preexec_fn=os.setsid)
    time.sleep(5)

    #Start tasks
    start_task(task)
    while True:
        time.sleep(1)


def exit_handler():
    global save_log
    if save_log:
        save_task()
    stop_task()
    time.sleep(2)
    global pro_task_selector
    assert isinstance(pro_task_selector, subprocess.Popen)
    pro_task_selector.kill()


atexit.register(exit_handler)


if __name__ == '__main__':
    if len(sys.argv) == 3 and sys.argv[2] == "--save":
        save_log = True
    main(sys.argv[1])