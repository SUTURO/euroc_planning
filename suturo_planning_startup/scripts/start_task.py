#!/usr/bin/env python
import os
import subprocess
import signal
import atexit
import time
import sys
from suturo_planning_task_selector import start_task, stop_task, save_task
from suturo_planning_plans.toplevel import toplevel_plan


pro_task_selector = 0

save_log = False


def main(task, with_plan):

    #Taskselector
    print "Starting task_selector"
    global pro_task_selector
    pro_task_selector = subprocess.Popen('rosrun euroc_launch TaskSelector', stdout=subprocess.PIPE,
                                         shell=True, preexec_fn=os.setsid)
    time.sleep(5)

    #If plans should be started start the state machine
    if with_plan:
        print 'Started plan'
        toplevel_plan()
    else:
        #Start tasks
        start_task(task)

        print 'Waiting for ctrl-c'
        while True:
            time.sleep(1)


def exit_handler():
    global save_log
    if save_log:
        save_task()
    stop_task()
    time.sleep(2)
    global pro_task_selector
    os.killpg(pro_task_selector.pid, signal.SIGTERM)


atexit.register(exit_handler)


if __name__ == '__main__':
    if '--save' in sys.argv:
        save_log = True
    if '--plan' in sys.argv:
        main(sys.argv[1], True)
    else:
        main(sys.argv[1], False)