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
__aborting_task = False
__kill_count = 0
__current_task = None
subproc = None


def print_it(s):
    print(str(datetime.now().isoformat('-')) + ': ' + str(s))


def abort_current_task():
    print_it('start_complete_demo: abort_current_task')
    global __aborting_task
    global subproc
    global __clock_subscriber
    global __quit
    if __quit:
        print_it('Already quitting. Nothing to do here.')
    elif __aborting_task:
        print_it ('Already aborting task.')
    else:
        __aborting_task = True
        if __clock_subscriber is not None:
            print_it('Unsubscribing clock subscriber.')
            __clock_subscriber.unregister()
        if subproc is not None:
            try:
                print_it('Killing subproc with pid ' + str(subproc.pid))
                exterminate(subproc.pid, signal.SIGINT)
                print_it('Waiting for subproc to terminate. Please be patient.')
                utils.wait_for_process(subproc, 90)
                poll = subproc.poll()
                if poll is None:
                    print_it('Could not kill task process. Forcing!'
                          ' (pid: ' + str(subproc.pid) + ', subproc.poll(): ' + str(poll) + ')')
                    exterminate(subproc.pid, signal.SIGKILL, r=True)
            except Exception, e:
                print_it(e)
        __aborting_task = False
    print_it('start_complete_demo: Exiting abort_current_task')


def kill_like_a_berserk():
    print_it('start_complete_demo: kill_like_a_berserk')
    global subproc
    print_it('Killing subproc like a berserk.')
    exterminate(subproc.pid, signal.SIGKILL, r=True)
    print_it('start_complete_demo: Exiting kill_like_a_berserk.')


def exit_handler(signum=None, frame=None):
    print_it('start_complete_demo: exit_handler')
    print_it('signum: ' + str(signum) + ', frame: ' + str(frame))
    if signum != signal.SIGINT:
        print_it('start_complete_demo: exit_handler: Unhandled signal. Exiting without any action.')
        return
    global __kill_count
    global __quit
    global __clock_subscriber
    global __aborting_task
    __kill_count += 1
    if __kill_count == 1:
        print_it('Going to abort current task.')
        abort_current_task()
    elif __kill_count == 2:
        print_it('Going to abort execution of all remaining tasks.')
        __quit = True
        if not __aborting_task:
            abort_current_task()
    elif __kill_count == 3:
        print_it('Going to kill like a berserk.')
        kill_like_a_berserk()
    else:
        print_it('You can stop spamming Ctrl-c now.')
    print_it('start_complete_demo: exiting exit_handler')


def start_demo(wait, tasks, logging):
    global __quit
    global __clock_subscriber
    global __time_started_task
    global __current_task
    global subproc
    rospy.init_node('start_complete_demo')

    rospy.loginfo('Setting use_sim_time to true')
    rospy.set_param('/use_sim_time', True)

    #atexit.register(exit_handler)
    signal.signal(signal.SIGTERM, exit_handler)
    signal.signal(signal.SIGINT, exit_handler)
    signal.signal(signal.SIGQUIT, exit_handler)
    print_it('Getting available task names.')
    tasks_to_execute = []
    task_names = get_available_task_names()
    print_it('Available task names: ' + str(task_names))
    #start_task.main(task_names[:2], True, True, True, True)
    if re.search('^\d*:\d*$', tasks):
        tasks_to_execute = eval('task_names[' + tasks + ']')
    elif re.search('^\d+$', tasks):
        tasks_to_execute = [task_names[int(tasks)]]
    elif re.search('^(\d+,?)+$', tasks):
        for num in tasks.split(','):
            tasks_to_execute.append(task_names[int(num)])
            tasks_to_execute.sort()
    else:
        for task in tasks.split(','):
            if task in task_names:
                tasks_to_execute.append(task)
            tasks_to_execute.sort()
    print_it('Going to execute the following tasks: ' + str(tasks_to_execute))
    if logging in [2]:
        dont_print = False
    else:
        dont_print = True
    for task in tasks_to_execute:
        rospy.loginfo('Setting use_sim_time to true')
        rospy.set_param('/use_sim_time', True)
        if __quit:
            print_it('Demo has been aborted. Exiting (1)')
            return
        init_time = __initialization_time + '-' + task
        __current_task = task
        if wait:
            raw_input('Starting task ' + str(task) + '. Press ENTER.')
        print_it('Starting task   ' + str(task))
        print_it('Finished tasks: ' + str(tasks_to_execute[0:tasks_to_execute.index(task)]))
        print_it('Tasks to go   : ' + str(tasks_to_execute[tasks_to_execute.index(task)+1:]))
        subproc, logger_process = utils.start_node('rosrun suturo_planning_startup start_task.py ' + task +
                                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"' +
                                                   ' --logging="' + str(logging) + '"', init_time, logging, 'Complete',
                                                   dont_print=dont_print, print_prefix_to_stdout=False)
        __time_started_task = int(time.time())
        print_it('Subscribing to clock.')
        __clock_subscriber = rospy.Subscriber('clock', Clock, handle_clock, callback_args=task)
        print_it('Waiting for task ' + task + ' to terminate.')
        subproc.wait()
        print_it('Task ' + task + ' terminated.')
        print_it('Unsubscribing clock subscriber.')
        __clock_subscriber.unregister()
        if __quit:
            print_it('Demo has been aborted. Exiting (2)')
            return
        print_it('Finished task ' + str(task))
        if task != tasks_to_execute[-1]:
            time.sleep(5)
    print_it('Finished complete demo.')
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


def handle_clock(msg, task):
    global __time_started_task
    global __time_limit
    global __aborting_task
    global __current_task
    now = int(time.time())
    if not __aborting_task and msg.clock.secs > __time_limit:
        if task == __current_task and now - __time_started_task >= __time_limit:
            print_panda()
            abort_current_task()
        else:
            print_it('Oh oh, received wrong information from clock. msg:')
            print_it(msg)
            print_it('__current_task: ' + str(__current_task))
            print_it('task: ' + str(task))
            print_it('now: ' + str(now))
            print_it('__time_started_task: ' + str(__time_started_task))
            print_it('__time_limit: ' + str(__time_limit))
            print_it('now - __time_started_task: ' + str(now - __time_started_task))


def print_panda():
    print_it('----------------------------------------------')
    print_it('| Ten minutes time limit has been succeeded. |')
    print_it('|             Terminating Task.              |')
    print_it('|                                            |')
    print_it('│  ─│─│───│─│───│───│─│─│─│───────│───│─│─   |')
    print_it('|  ─│─│──╫▓▓▓╫──│─────│─│─│──────╫▓▓╫│──│─│  |')
    print_it('|  ──│─▓███████▓─╫╫╫╫╫╫╫╫╫╫╫╫╫│▓███████╫──   |')
    print_it('|  ───██████████████████████████████████▓─   |')
    print_it('|  │─████████████│─│─│─│─────▓███████████╫   |')
    print_it('|  ─╫███████▓╫││╫─────│───│─││╫││╫████████│  |')
    print_it('|  ─▓██████────│─│───────│─│─│─│─│─╫██████│  |')
    print_it('|  ─██████│─│───│───│─│─│───│───────│█████▓  |')
    print_it('|  ╫█████────│───│─│───│─────│─│─│───╫████▓  |')
    print_it('|  │████▓─│─│─│───│───│─────│───│─────████▓  |')
    print_it('|  │████│──│───│───│─│───│───────│─│─│▓███╫  |')
    print_it('|  ─▓███│───────│─▓██───│╫██╫─│─│─│───▓███│  |')
    print_it('|  ──███─│──────╫████▓───█████────────▓███─  |')
    print_it('|  ──╫██──│─│──╫██████│─│██████─│─────▓██─│  |')
    print_it('|  │─│▓█││─│─││███▓▓██─│─██▓▓███─│─│──▓█─│─  |')
    print_it('|  ────█│─│───███╫▓▓█▓│──█▓▓▓▓██▓─────▓█───  |')
    print_it('|  │─││█││───▓███╫██▓╫─│─▓▓█▓▓███─────▓█───  |')
    print_it('|  ─│─╫█│─│─│████▓╫▓▓─────█▓╫████▓──│─▓█───  |')
    print_it('|  │─││█╫│─││███████─│██╫│▓███████─│─│██─│─  |')
    print_it('|  ─│─│█▓╫╫─▓██████╫│─▓█│──▓██████│╫╫│██│─│  |')
    print_it('|  │─│─██│╫│▓█████╫│───▓───│▓█████╫╫╫╫█▓──   |')
    print_it('|  ─│─│▓█╫││╫████╫│││╫██▓││││▓████│╫─▓█╫│─│  |')
    print_it('|  │─│─│██│││╫▓▓││╫╫╫╫╫▓╫╫╫╫╫│╫▓▓╫││╫██──│─  |')
    print_it('|  ─│───▓██╫─────││││││─││││││────│▓██│────  |')
    print_it('|  │─│─│─▓██▓╫╫╫╫╫╫╫╫▓▓▓▓▓╫╫╫╫╫╫╫▓███│────   |')
    print_it('|  ───────╫██████████▓▓▓▓▓██████████│────│   |')
    print_it('|  │─│─│───▓█████████╫─│─▓█████████│─│─│─│   |')
    print_it('|  ─────────██████████──│█████████╫─│───││   |')
    print_it('|  │─│─│───│▓█╫███████││▓███████╫█││─│─│─│   |')
    print_it('|  ───────│─██─╫██████▓─███████││█╫───│──│   |')
    print_it('|  │───│───│██─││█████▓─█████▓─│╫█╫│──────   |')
    print_it('|  ─│─│───│─▓█──│─╫▓██│─▓██▓│─│─▓█│───────   |')
    print_it('|  │───│─│─│─██────│─│───│─────│██───│─│─│   |')
    print_it('|  ─│─│───│─│▓██╫─│─│─────│─│─▓██││─│───│─│  |')
    print_it('|  │───────│─│██████████████████▓│─│─│─│─│   |')
    print_it('|  ─│───│─│───│███████▓▓████████│─│───│──│   |')
    print_it('|  │─│───│─│─│─│██████╫─▓█████▓────│─│─│──   |')
    print_it('|  ─────│─────╫│╫▓████▓─█████▓│╫╫───────│    |')
    print_it('|  │─│───│───╫─╫╫╫╫███╫╫╫██▓╫│╫╫╫│─│─────    |')
    print_it('|  ───│─│──────││───────│─│───│─│─│───│─│    |')
    print_it('|                                            |')
    print_it('|                                            |')
    print_it('|             Sad panda is sad.              |')
    print_it('|                                            |')
    print_it('----------------------------------------------')


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