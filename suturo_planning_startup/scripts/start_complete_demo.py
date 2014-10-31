#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import getopt
import sys
import signal
import re
import rospy
import atexit
from rosgraph_msgs.msg import Clock
from suturo_planning_manipulation.total_annihilation import exterminate
from suturo_planning_plans import utils
from datetime import datetime
from euroc_c2_msgs.srv import ListScenes

__initialization_time = datetime.now().isoformat('-')
__quit = False
__clock_subscriber = None
__time_started_task = None
__time_limit = 600
__handling_exit = False
subproc = None
logger_process = None


def abort_current_task():
    print('start_complete_demo: abort_current_task')
    global subproc
    global logger_process
    if subproc is not None:
        try:
            print('Killing subproc with pid ' + str(subproc.pid))
            exterminate(subproc.pid, signal.SIGINT)
            print('Waiting for subproc to terminate. Please be patient.')
            subproc.wait()
            if subproc.poll() is None:
                print('Could not kill task process. Forcing!')
                exterminate(subproc.pid, signal.SIGKILL)
        except Exception, e:
            print(e)
    if logger_process is not None:
        try:
            print('Killing logger with pid ' + str(logger_process.pid))
            exterminate(logger_process.pid, signal.SIGINT)
            print('Waiting for logger process to terminate. Please be patient.')
            logger_process.wait()
            #utils.wait_for_process(logger_process, 20)
            #if logger_process.poll() is None:
                #print('Could not kill logger process. Forcing!')
                #exterminate(logger_process.pid, signal.SIGKILL)
        except Exception, e:
            print(e)


def exit_handler(signum=None, frame=None):
    print('start_complete_demo: exit_handler')
    global __handling_exit
    if __handling_exit:
        print('start_complete_demo: Already handling exit.')
        return
    __handling_exit = True
    global __quit
    global __clock_subscriber
    if __clock_subscriber is not None:
        print('Unregistering clock subscriber.')
        __clock_subscriber.unregister()
    if signum is not None:
        abort_current_task()
    __quit = True
    print('start_complete_demo: exiting exit_handler')


def start_demo(wait, tasks, logging):
    global __quit
    global __clock_subscriber
    global __time_started_task
    global subproc
    global logger_process
    rospy.init_node('start_complete_demo')
    atexit.register(exit_handler)
    signal.signal(signal.SIGTERM, exit_handler)
    signal.signal(signal.SIGINT, exit_handler)
    print('Getting available task names.')
    tasks_to_execute = []
    task_names = get_available_task_names()
    print('Available task names: ' + str(task_names))
    #start_task.main(task_names[:2], True, True, True, True)
    if re.search('^\d*:\d*$', tasks):
        tasks_to_execute = eval('task_names[' + tasks + ']')
    elif re.search('^\d+$', tasks):
        tasks_to_execute = [task_names[int(tasks)]]
    elif re.search('^(\d+,?)+$', tasks):
        for num in tasks.split(','):
            tasks_to_execute.append(task_names[int(num)])
    else:
        for task in tasks.split(','):
            if task in task_names:
                tasks_to_execute.append(task)
    print('Going to execute the following tasks: ' + str(tasks_to_execute))
    if logging in [2]:
        dont_print = False
    else:
        dont_print = True
    for task in tasks_to_execute:
        if __quit:
            print('Demo has been aborted. Exiting (1)')
            return
        init_time = __initialization_time + '-' + task
        if wait:
            raw_input('Starting task ' + str(task) + '. Press ENTER.')
        print('Starting task ' + str(task))
        print('Tasks to go: ' + str(tasks_to_execute[tasks_to_execute.index(task)+1:]))
        global subproc
        global logger_process
        subproc, logger_process = utils.start_node('rosrun suturo_planning_startup start_task.py ' + task +
                                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"' +
                                                   ' --logging="' + str(logging) + '"', init_time, logging, 'Complete',
                                                   dont_print=dont_print, print_prefix_to_stdout=False)
        __time_started_task = int(time.time())
        print('Subscribing to clock.')
        __clock_subscriber = rospy.Subscriber('clock', Clock, handle_clock)
        print('Waiting for task ' + task + ' to terminate.')
        subproc.wait()
        if logger_process is not None and logger_process.poll() is None:
            print('Waiting for logger to terminate.')
            exterminate(logger_process.pid, signal.SIGINT)
            logger_process.wait()
        if __quit:
            print('Demo has been aborted. Exiting (2)')
            return
        print('Finished task ' + str(task))
    rospy.signal_shutdown('Finished complete demo.')


def get_available_tasks():
    rospy.wait_for_service('euroc_c2_task_selector/list_scenes')
    service = rospy.ServiceProxy('euroc_c2_task_selector/list_scenes', ListScenes)
    return service().scenes


def get_available_task_names():
    available_tasks = get_available_tasks()
    task_names = []
    for t in available_tasks:
        task_names.append(t.name)
    task_names.sort()
    return task_names


def handle_clock(msg):
    global __clock_subscriber
    global __time_started_task
    global __time_limit
    now = int(time.time())
    if msg.clock.secs > __time_limit:
        if now - __time_started_task >= __time_limit:
            print('----------------------------------------------')
            print('| Ten minutes time limit has been succeeded. |')
            print('|             Terminating Task.              |')
            print('|                                            |')
            print('│  ─│─│───│─│───│───│─│─│─│───────│───│─│─   |')
            print('|  ─│─│──╫▓▓▓╫──│─────│─│─│──────╫▓▓╫│──│─│  |')
            print('|  ──│─▓███████▓─╫╫╫╫╫╫╫╫╫╫╫╫╫│▓███████╫──   |')
            print('|  ───██████████████████████████████████▓─   |')
            print('|  │─████████████│─│─│─│─────▓███████████╫   |')
            print('|  ─╫███████▓╫││╫─────│───│─││╫││╫████████│  |')
            print('|  ─▓██████────│─│───────│─│─│─│─│─╫██████│  |')
            print('|  ─██████│─│───│───│─│─│───│───────│█████▓  |')
            print('|  ╫█████────│───│─│───│─────│─│─│───╫████▓  |')
            print('|  │████▓─│─│─│───│───│─────│───│─────████▓  |')
            print('|  │████│──│───│───│─│───│───────│─│─│▓███╫  |')
            print('|  ─▓███│───────│─▓██───│╫██╫─│─│─│───▓███│  |')
            print('|  ──███─│──────╫████▓───█████────────▓███─  |')
            print('|  ──╫██──│─│──╫██████│─│██████─│─────▓██─│  |')
            print('|  │─│▓█││─│─││███▓▓██─│─██▓▓███─│─│──▓█─│─  |')
            print('|  ────█│─│───███╫▓▓█▓│──█▓▓▓▓██▓─────▓█───  |')
            print('|  │─││█││───▓███╫██▓╫─│─▓▓█▓▓███─────▓█───  |')
            print('|  ─│─╫█│─│─│████▓╫▓▓─────█▓╫████▓──│─▓█───  |')
            print('|  │─││█╫│─││███████─│██╫│▓███████─│─│██─│─  |')
            print('|  ─│─│█▓╫╫─▓██████╫│─▓█│──▓██████│╫╫│██│─│  |')
            print('|  │─│─██│╫│▓█████╫│───▓───│▓█████╫╫╫╫█▓──   |')
            print('|  ─│─│▓█╫││╫████╫│││╫██▓││││▓████│╫─▓█╫│─│  |')
            print('|  │─│─│██│││╫▓▓││╫╫╫╫╫▓╫╫╫╫╫│╫▓▓╫││╫██──│─  |')
            print('|  ─│───▓██╫─────││││││─││││││────│▓██│────  |')
            print('|  │─│─│─▓██▓╫╫╫╫╫╫╫╫▓▓▓▓▓╫╫╫╫╫╫╫▓███│────   |')
            print('|  ───────╫██████████▓▓▓▓▓██████████│────│   |')
            print('|  │─│─│───▓█████████╫─│─▓█████████│─│─│─│   |')
            print('|  ─────────██████████──│█████████╫─│───││   |')
            print('|  │─│─│───│▓█╫███████││▓███████╫█││─│─│─│   |')
            print('|  ───────│─██─╫██████▓─███████││█╫───│──│   |')
            print('|  │───│───│██─││█████▓─█████▓─│╫█╫│──────   |')
            print('|  ─│─│───│─▓█──│─╫▓██│─▓██▓│─│─▓█│───────   |')
            print('|  │───│─│─│─██────│─│───│─────│██───│─│─│   |')
            print('|  ─│─│───│─│▓██╫─│─│─────│─│─▓██││─│───│─│  |')
            print('|  │───────│─│██████████████████▓│─│─│─│─│   |')
            print('|  ─│───│─│───│███████▓▓████████│─│───│──│   |')
            print('|  │─│───│─│─│─│██████╫─▓█████▓────│─│─│──   |')
            print('|  ─────│─────╫│╫▓████▓─█████▓│╫╫───────│    |')
            print('|  │─│───│───╫─╫╫╫╫███╫╫╫██▓╫│╫╫╫│─│─────    |')
            print('|  ───│─│──────││───────│─│───│─│─│───│─│    |')
            print('|                                            |')
            print('|                                            |')
            print('|             Sad panda is sad.              |')
            print('|                                            |')
            print('----------------------------------------------')
            __clock_subscriber.unregister()
            abort_current_task()
        else:
            print('Oh oh, received wrong information from clock:')
            print(msg)


def main(argv):
    usage = 'usage: start_complete_demo [-t] [-w] [--tasks=<tasks you want to execute>] [--logging=<logging mode>]\n\n' \
            '\t-t\t\tPrints a list of all available tasks.\n' \
            '\t-w\t\tThe user has to press ENTER after each task.\n' \
            '\t--tasks\t\tA range like \':2\' (python slice and index notation)\n' \
            '\t\t\tor a comma-separated list of task names\n' \
            '\t\t\tor a comma-separated list of indices (as shown by -t).\n' \
            '\t--logging\t0 - Logs to files only (Default value).\n' \
            '\t\t\t1 - Logs to console only.\n' \
            '\t\t\t2 - Logs to files and console.'
    wait = False
    tasks = ':'
    logging = 0
    try:
        opts, args = getopt.getopt(argv, 'hwct', ['tasks=', 'logging='])
        #print ('opts: ' + str(opts))
        #print ('args: ' + str(args))
    except getopt.GetoptError:
        print usage
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print usage
            sys.exit()
        elif opt == '-t':
            task_names = get_available_task_names()
            print('Available tasks:\n')
            print('  Index     Task name')
            print(' ---------------------------')
            i = 0
            for t in task_names:
                print('  %3d       %s' % (i, t))
                i += 1
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