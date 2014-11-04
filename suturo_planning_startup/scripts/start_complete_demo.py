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
__aborting_task = False
__kill_count = 0
subproc = None
logger_process = None


def abort_current_task():
    print('start_complete_demo: abort_current_task')
    global __aborting_task
    global subproc
    global logger_process
    global __clock_subscriber
    global __quit
    if __quit:
        print('Already quitting. Nothing to do here.')
        return
    elif __aborting_task:
        print ('Already aborting task.')
    else:
        __aborting_task = True
        if __clock_subscriber is not None:
            print('Unsubscribing clock subscriber.')
            __clock_subscriber.unregister()
        if subproc is not None:
            try:
                print('Killing subproc with pid ' + str(subproc.pid))
                exterminate(subproc.pid, signal.SIGINT)
                print('Waiting for subproc to terminate. Please be patient.')
                utils.wait_for_process(subproc, 120)
                poll = subproc.poll()
                if poll is None:
                    print('Could not kill task process. Forcing!'
                          ' (pid: ' + str(subproc.pid) + ', subproc.poll(): ' + str(poll) + ')')
                    exterminate(subproc.pid, signal.SIGKILL, r=True)
            except Exception, e:
                print(e)
        if logger_process is not None:
            try:
                print('Killing logger with pid ' + str(logger_process.pid))
                exterminate(logger_process.pid, signal.SIGINT)
                print('Waiting for logger process to terminate. Please be patient.')
                #logger_process.wait()
                utils.wait_for_process(logger_process, 240)
                poll = logger_process.poll()
                if poll is None:
                    print('Could not kill logger process. Forcing!'
                          ' (pid: ' + str(logger_process.pid) + ', logger_process.poll(): ' + str(poll) + ')')
                    exterminate(logger_process.pid, signal.SIGKILL)
            except Exception, e:
                print(e)
        __aborting_task = False
    print('start_complete_demo: Exiting abort_current_task')


def kill_like_a_berserk():
    print('start_complete_demo: kill_like_a_berserk')
    global subproc
    global logger_process
    print('Killing subproc like a berserk.')
    exterminate(subproc.pid, signal.SIGKILL, r=True)
    print('Killing logger like a berserk.')
    exterminate(logger_process.pid, signal.SIGKILL, r=True)
    print('start_complete_demo: Exiting kill_like_a_berserk.')


def exit_handler(signum=None, frame=None):
    global __kill_count
    global __handling_exit
    print('start_complete_demo: exit_handler')
    global __quit
    global __clock_subscriber
    global __aborting_task
    if __handling_exit:
        if not __quit:
            print('Going to abort execution of all remaining tasks.')
            __quit = True
            if not __aborting_task:
                abort_current_task()
        else:
            print('start_complete_demo: Already handling exit. Count to kill like a berserk: ' + str(__kill_count))
            if __kill_count < 100:
                __kill_count += 1
            elif signum is not None and __kill_count == 100:
                __kill_count += 1
                kill_like_a_berserk()
        return
    else:
        __handling_exit = True
        if signum is not None:
            print('Going to abort current task.')
            abort_current_task()
        if not __quit:
            print('Setting __handling_exit to False.')
            __handling_exit = False
    print('start_complete_demo: exiting exit_handler')


def start_demo(wait, tasks, logging):
    global __quit
    global __clock_subscriber
    global __time_started_task
    global __aborting_task
    global subproc
    global logger_process
    rospy.init_node('start_complete_demo')
    #atexit.register(exit_handler)
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
        subproc, logger_process = utils.start_node('rosrun suturo_planning_startup start_task.py ' + task +
                                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"' +
                                                   ' --logging="' + str(logging) + '"', init_time, logging, 'Complete',
                                                   dont_print=dont_print, print_prefix_to_stdout=False)
        __time_started_task = int(time.time())
        print('Subscribing to clock.')
        __clock_subscriber = rospy.Subscriber('clock', Clock, handle_clock)
        print('Waiting for task ' + task + ' to terminate.')
        subproc.wait()
        print('Task ' + task + ' terminated.')
        print('Unsubscribing clock subscriber.')
        __clock_subscriber.unregister()
        if logger_process is not None and logger_process.poll() is None:
            if __aborting_task:
                print('Already aborting task. Won\'t kill logger.')
                print('Waiting for abortion to be finished.')
                while __aborting_task:
                    time.sleep(1)
                print('Abortion finished.')
            else:
                print('Killing logger.')
                exterminate(logger_process.pid, signal.SIGINT)
                print('Waiting for logger to terminate.')
                utils.wait_for_process(logger_process, 240)
                poll = logger_process.poll()
                if poll is None:
                    print('Could not kill logger process. Forcing!'
                          ' (pid: ' + str(logger_process.pid) + ', logger_process.poll(): ' + str(poll) + ')')
                    exterminate(logger_process.pid, signal.SIGKILL)
        if __quit:
            print('Demo has been aborted. Exiting (2)')
            return
        print('Finished task ' + str(task))
        if task != tasks_to_execute[-1]:
            time.sleep(5)
    print('Finished complete demo.')
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
    global __time_started_task
    global __time_limit
    global __aborting_task
    now = int(time.time())
    if not __aborting_task and msg.clock.secs > __time_limit:
        if now - __time_started_task >= __time_limit:
            print_panda()
            abort_current_task()
        else:
            print('Oh oh, received wrong information from clock. msg:')
            print(msg)
            print('now: ' + str(now))
            print('__time_started_task: ' + str(__time_started_task))
            print('__time_limit: ' + str(__time_limit))


def print_panda():
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


def main(argv):
    usage = 'usage: start_complete_demo [-t] [-w] [--tasks=<tasks you want to execute>] [--logging=<logging mode>]\n\n'\
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
        opts, args = getopt.getopt(argv, 'hwct', ['tasks=', 'logging=', 'help'])
        #print ('opts: ' + str(opts))
        #print ('args: ' + str(args))
    except getopt.GetoptError:
        print usage
        sys.exit(2)
    for opt, arg in opts:
        if opt in ['-h', '--help']:
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