#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import getopt
import sys
import signal
import re
import rospy
import os
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
__clock_time_started_task = None
__remaining_time = None
__last_received_clock = None
__aborting_task = False
__kill_count = 0
__current_task = None
__current_intent = 1
__task_aborted_by_exception = False
__after_task = False
subproc = None


def print_it(s):
    print(str(datetime.now().isoformat('-')) + ': ' + str(s))


def usr1_handler(signum=None, frame=None):
    print_it('start_complete_demo: usr1_handler')
    global __task_aborted_by_exception
    global __aborting_task
    if not __aborting_task:
        print_it('Setting __task_aborted_by_exception to True.')
        __task_aborted_by_exception = True


def usr2_handler(signum=None, frame=None):
    print_it('start_complete_demo: usr2_handler')
    global __quit
    global __kill_count
    if not __quit:
        print_it('####################################################')
        print_it('# Going to abort execution of all remaining tasks. #')
        print_it('####################################################')
        __quit = True
        __kill_count = 1337
        if not __aborting_task:
            abort_current_task()


def abort_current_task():
    print_it('start_complete_demo: abort_current_task')
    global __aborting_task
    global subproc
    global __clock_subscriber
    global __quit
    if __quit:
        print_it('Already quitting. Nothing to do here.')
    elif __aborting_task:
        print_it('Already aborting task. Nothing to do here.')
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
        print_it('################################')
        print_it('# Going to abort current task. #')
        print_it('################################')
        abort_current_task()
    elif __kill_count == 2:
        print_it('####################################################')
        print_it('# Going to abort execution of all remaining tasks. #')
        print_it('####################################################')
        __quit = True
        if not __aborting_task:
            abort_current_task()
    elif __kill_count == 3:
        print_it('#################################')
        print_it('# Going to kill like a berserk. #')
        print_it('#################################')
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
    global __kill_count
    global __remaining_time
    global __time_limit
    global __task_aborted_by_exception
    global __current_intent
    global __clock_time_started_task
    global __last_received_clock
    global __after_task
    rospy.init_node('start_complete_demo')

    rospy.loginfo('Setting use_sim_time to true')
    rospy.set_param('/use_sim_time', True)

    #atexit.register(exit_handler)
    signal.signal(signal.SIGTERM, exit_handler)
    signal.signal(signal.SIGINT, exit_handler)
    signal.signal(signal.SIGQUIT, exit_handler)
    signal.signal(signal.SIGUSR1, usr1_handler)
    signal.signal(signal.SIGUSR2, usr2_handler)
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
    i = 0
    __clock_time_started_task = 0
    while i < len(tasks_to_execute):
        __kill_count = 0
        __task_aborted_by_exception = False
        rospy.loginfo('Setting use_sim_time to true')
        rospy.set_param('/use_sim_time', True)
        if __quit:
            print_it('Demo has been aborted. Exiting (1)')
            return
        __current_task = tasks_to_execute[i]
        init_time = __initialization_time + '-' + __current_task + '-' + str(__current_intent)
        if wait:
            raw_input('Starting task ' + str(__current_task) + '. Press ENTER.')
        print_it('################################################################################')
        print_it('Starting task : ' + str(__current_task))
        print_it('Intent        : ' + str(__current_intent))
        print_it('Started at    : ' + str(__clock_time_started_task))
        print_it('Finished tasks: ' + str(tasks_to_execute[0:tasks_to_execute.index(__current_task)]))
        print_it('Tasks to go   : ' + str(tasks_to_execute[tasks_to_execute.index(__current_task)+1:]))
        print_it('################################################################################')
        subproc, logger_process = utils.start_node('rosrun suturo_planning_startup start_task.py ' + __current_task +
                                                   ' --plan --init --save --no-ts --inittime="' + init_time + '"' +
                                                   ' --logging="' + str(logging) + '"' + ' --parent=' + str(os.getpid())
                                                   , init_time, logging, 'Complete',
                                                   dont_print=dont_print, print_prefix_to_stdout=False)
        __time_started_task = int(time.time())
        print_it('Subscribing to clock.')
        __clock_subscriber = rospy.Subscriber('clock', Clock, handle_clock, queue_size=1,
                                              callback_args={'task': __current_task,
                                                             'intent': __current_intent})
        print_it('Waiting for task ' + __current_task + ' to terminate.')
        __after_task = False
        subproc.wait()
        __after_task = True
        print_it('Task ' + __current_task + ' terminated')
        print_it('Unsubscribing clock subscriber.')
        __clock_subscriber.unregister()
        if __quit:
            print_it('Demo has been aborted. Exiting (2)')
            return
        else:
            if __task_aborted_by_exception:
                print_it('Task has been interrupted by exception.')
                if __remaining_time > __time_limit / 2:
                    print_it('Restarting task ' + __current_task)
                    __current_intent += 1
                    __clock_time_started_task += __last_received_clock
                else:
                    print_it('Remaining time (' + str(__remaining_time) + 's) is too low to restart task.')
                    print_it('Finished task ' + str(__current_task))
                    i += 1
                    __current_intent = 1
                    __clock_time_started_task = 0
            else:
                print_it('Task terminated as expected.')
                print_it('Finished task ' + str(__current_task))
                i += 1
                __current_intent = 1
                __clock_time_started_task = 0
                if i != len(tasks_to_execute):
                    time.sleep(5)
            # __remaining_time = __time_limit
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


def handle_clock(msg, args):
    global __time_started_task
    global __time_limit
    global __aborting_task
    global __current_task
    global __current_intent
    global __remaining_time
    global __last_received_clock
    global __clock_time_started_task
    global __after_task
    now = int(time.time())
    clock_time = msg.clock.secs
    remaining_time = __time_limit - __clock_time_started_task - clock_time
    task = args['task']
    intent = args['intent']

    if task == __current_task and intent == __current_intent:
        __last_received_clock = clock_time
        if not __aborting_task:
            __remaining_time = remaining_time

            def print_info():
                print_it(msg)
                print_it('__current_task: ' + str(__current_task))
                print_it('__current_intent: ' + str(__current_intent))
                print_it('task: ' + str(task))
                print_it('intent: ' + str(intent))
                print_it('remaining_time: ' + str(remaining_time))
                print_it('__clock_time_started_task: ' + str(__clock_time_started_task))
                print_it('__time_limit: ' + str(__time_limit))
                print_it('now: ' + str(now))
                print_it('__time_started_task: ' + str(__time_started_task))
                print_it('now - __time_started_task: ' + str(now - __time_started_task))

            if remaining_time <= 0 and not __after_task:
                if now - __time_started_task >= __time_limit:
                    print_it('handle_clock:')
                    print_info()
                    print_panda()
                    abort_current_task()
                else:
                    print_it('Oh oh, received wrong information (old data from previous task) from clock.')
                    print_info()


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