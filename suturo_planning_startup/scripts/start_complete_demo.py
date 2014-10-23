#!/usr/bin/env python

import getopt
import sys
import signal
import os
import re
import rospy
import atexit
from suturo_planning_plans import utils
from datetime import datetime
from euroc_c2_msgs.srv import ListScenes
import subprocess

__initialization_time = datetime.now().isoformat(' ')
subproc = 0
logger_process = 0


def exit_handler():
    try:
        os.killpg(subproc.pid, signal.SIGTERM)
        os.killpg(logger_process.pid, signal.SIGTERM)
    except:
        pass

atexit.register(exit_handler)


def start_demo(wait, tasks):
    print('Getting available task names.')
    rospy.wait_for_service('euroc_c2_task_selector/list_scenes')
    service = rospy.ServiceProxy('euroc_c2_task_selector/list_scenes', ListScenes)
    available_tasks = service()
    task_names = []
    tasks_to_execute = []
    for t in available_tasks.scenes:
        task_names.append(t.name)
    task_names.sort()
    print('Available task names: ' + str(task_names))
    #start_task.main(task_names[:2], True, True, True, True)
    if re.search('^\d*:?\d*$', tasks):
        tasks_to_execute = eval('task_names[' + tasks + ']')
    else:
        for task in tasks.split(','):
            if task in task_names:
                tasks_to_execute.append(task)

    print('Going to execute the following tasks: ' + str(tasks_to_execute))
    for task in tasks_to_execute:
        init_time = __initialization_time + ' ' + task
        if wait:
            raw_input('Starting task ' + str(task) + '. Press ENTER.')
        print('Starting task ' + str(task))
        global subproc
        subproc = subprocess.Popen('rosrun suturo_planning_startup start_task.py ' + task +
                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"',
                                   shell=True, stderr=subprocess.STDOUT, stdout=subprocess.PIPE)
        print('Starting logger.')
        global logger_process
        logger_process = subprocess.Popen('./logger.py "' + utils.log_dir + '/' + init_time +
                                          ' Complete.log"', stdin=subproc.stdout, shell=True)
        subproc.wait()
        print('Killing logger.')
        try:
            os.kill(logger_process.pid, signal.SIGTERM)
        except:
            print('Could not kill logger with pid ' + str(logger_process.pid))
            print('Trying to force kill.')
            try:
                os.kill(logger_process.pid, signal.SIGINT)
            except:
                print('Still not possible to kill logger.')

def main(argv):
    usage = 'usage: start_complete_demo [--tasks=<tasks you want to execute> [-w]\n\n' \
            '\t--tasks\t\tA range like \'[:2]\' or a comma-separated list of task names.\n' \
            '\t-w\t\tThe user has to press ENTER after each task.'
    wait = False
    tasks = ':'
    try:
        opts, args = getopt.getopt(argv, 'hw', ['tasks='])
        #print ('opts: ' + str(opts))
        #print ('args: ' + str(args))
    except getopt.GetoptError:
        print usage
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print usage
            sys.exit()
        elif opt in ['-w']:
            wait = True
        elif opt in ['--tasks']:
            tasks = arg
    start_demo(wait, tasks)

if __name__ == '__main__':
    main(sys.argv[1:])