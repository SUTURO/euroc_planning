#!/usr/bin/env python
import time
import getopt
import sys
import signal
import re
import rospy
import atexit
from suturo_planning_manipulation.total_annihilation import exterminate
from suturo_planning_plans import utils
from datetime import datetime
from euroc_c2_msgs.srv import ListScenes

__initialization_time = datetime.now().isoformat(' ')
__quit = False
subproc = None
logger_process = None


def exit_handler(signum=None, frame=None):
    print('start_complete_demo: exit_handler')
    global __quit
    global subproc
    global logger_process
    try:
        print('Killing subproc with pid ' + str(subproc.pid))
        exterminate(subproc.pid, signal.SIGINT)
        time.sleep(15)
        if subproc.poll() is None:
            print('Could not kill task process. Forcing!')
            exterminate(subproc.pid, signal.SIGKILL)
    except:
        pass
    try:
        print('Killing logger with pid ' + str(logger_process.pid))
        exterminate(logger_process.pid, signal.SIGINT)
    except:
        pass
    __quit = True
    print('start_complete_demo: exiting exit_handler')

atexit.register(exit_handler)
signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)


def start_demo(wait, tasks, logging):
    global __quit
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
    if len(tasks_to_execute) == 0:
        tasks_to_execute = task_names
    print('Going to execute the following tasks: ' + str(tasks_to_execute))
    if logging in [2]:
        dont_print = False
    else:
        dont_print = True
    for task in tasks_to_execute:
        if __quit:
            print('Demo has been aborted. Exiting (1)')
            return
        init_time = __initialization_time + ' ' + task
        if wait:
            raw_input('Starting task ' + str(task) + '. Press ENTER.')
        print('Starting task ' + str(task))
        print('Tasks to go: ' + str(tasks_to_execute[tasks_to_execute.index(task)+1:]))
        global subproc
        global logger_process
        subproc, logger_process = utils.start_node('rosrun suturo_planning_startup start_task.py ' + task +
                                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"' +
                                                   ' --logging="' + str(logging) + '"', init_time, 'Complete', logging,
                                                   dont_print=dont_print, print_prefix_to_stdout=False)
        print('Waiting for task ' + task + ' to terminate.')
        subproc.wait()
        print('Waiting for logger to terminate.')
        exterminate(logger_process.pid, signal.SIGINT)
        logger_process.wait()
        if __quit:
            print('Demo has been aborted. Exiting (2)')
            return
        print('Finished task ' + str(task))
        if logging != 1:
            print('Killing logger.')
            exterminate(logger_process.pid, signal.SIGINT)


def main(argv):
    usage = 'usage: start_complete_demo [--tasks=<tasks you want to execute>] [--logging=<logging mode>] [-w]\n\n' \
            '\t--tasks\t\tA range like \'[:2]\' or a comma-separated list of task names.\n' \
            '\t--logging\t0 - Logs to files only (Default value).\n' \
            '\t\t\t1 - Logs to console only.\n' \
            '\t\t\t2 - Logs to files and console.\n' \
            '\t-w\t\tThe user has to press ENTER after each task.\n'
    wait = False
    tasks = ':'
    logging = 0
    try:
        opts, args = getopt.getopt(argv, 'hwc', ['tasks=', 'logging='])
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
        elif opt in ['--logging']:
            l = int(arg)
            if l in [0, 1, 2]:
                logging = l
            else:
                print usage
                sys.exit(2)
        elif opt in ['--tasks']:
            tasks = arg
    start_demo(wait, tasks, logging)

if __name__ == '__main__':
    main(sys.argv[1:])