import traceback
import os
import sys
import rospy
import smach
import smach_ros
import threading
import subprocess
import time
from suturo_planning_plans import task1_2
import task1
import task4
import task6
import start_nodes
import signal
import atexit
from suturo_planning_manipulation.total_annihilation import exterminate
from suturo_planning_task_selector import task_selector
from suturo_msgs.msg import Task
from suturo_planning_plans import utils
from suturo_planning_plans.utils import secure_log

__logger_process = None
__handling_exit = False


def exit_handler(signum=None, frame=None):
    print('toplevel: exit_handler')
    global __handling_exit
    if __handling_exit:
        print('toplevel: Already handling exit.')
        return
    __handling_exit = True
    if __logger_process is not None:
        print 'Killing planning logger process.'
        exterminate(__logger_process.pid, signal.SIGINT)
    print('toplevel: Exiting exit_handler.')

signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)


def handle_uncaught_exception(e, initialization_time, logging):
    print('Uncaught exception: ' + str(e))
    if not start_nodes.executed_test_node_check:
        start_nodes.check_node(initialization_time, logging)
    print('Terminating task.')
    rospy.signal_shutdown('Terminating Task due to unhandled exception.')


def toplevel_plan(init_sim, task_name, savelog, initialization_time, logging):
    #Create a SMACH state machine
    global __logger_process
    print('toplevel: logging: ' + str(logging))
    if logging == 1:
        print('toplevel: Logging planning to console.')
    else:
        print('toplevel: Logging planning to files.')
        __logger_process = utils.start_logger(os.getpid(), subprocess.PIPE, initialization_time, 'Planning', logging)
        sys.stderr = __logger_process.stdin
        sys.stdout = __logger_process.stdin

    toplevel = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with toplevel:
        rospy.logdebug('Adding task: %s', task_name)
        smach.StateMachine.add('Execute%s' % task_name,
                               EurocTask(init_sim, task_name, savelog, initialization_time, logging),
                               transitions={'success': 'success',
                                            'fail': 'fail'})

    # Create and start the introspection server
    rospy.loginfo('Creating and starting the introspection server.')
    sis = smach_ros.IntrospectionServer('toplevel', toplevel, '/SM_ROOT')
    sis.start()

    # Create a thread to execute the smach container
    rospy.loginfo('Creating a thread to execute the smach container.')

    def execute_task():
        try:
            toplevel.execute()
        except BaseException, e:
            print('BaseException while executing Task:')
            print(traceback.print_exc())
            handle_uncaught_exception(sys.exc_info()[0], initialization_time, logging)
        except Exception, e:
            print('Exception while executing Task:')
            print(traceback.print_exc())
            handle_uncaught_exception(sys.exc_info()[0], initialization_time, logging)
        except:
            print('Unhandled Exception while executing Task:')
            print(traceback.print_exc())
            handle_uncaught_exception(sys.exc_info()[0], initialization_time, logging)

    smach_thread = threading.Thread(target=execute_task)
    rospy.loginfo('toplevel: Starting smach thread.')
    smach_thread.start()
    rospy.loginfo('toplevel: Waiting for smach thread to terminate.')
    # Wait for ctrl-c
    rospy.spin()
    secure_log(rospy.loginfo, 'toplevel: smach thread terminated.')
    secure_log(rospy.loginfo, 'toplevel: Interrupting smach thread.')

    # # Request the container to preempt
    # rospy.loginfo('Request the container to preempt.')
    # toplevel.request_preempt()
    #
    # # Block until everything is preempted
    # rospy.loginfo('Block until everything is preempted.')
    secure_log(rospy.loginfo, 'toplevel: Joining smach thread.')
    start_nodes.exit_handler()
    smach_thread.join()
    secure_log(rospy.loginfo, 'toplevel: smach thread terminated.')


class EurocTask(smach.StateMachine):
    def __init__(self, init_sim, task_name, savelog, initialization_time, logging):
        self.savelog = savelog
        smach.StateMachine.__init__(self, input_keys=[], outcomes=['success', 'fail'])
        self.userdata.initialization_time = initialization_time
        self.userdata.logging = logging

        available_tasks = ['task1', 'task2', 'task3', 'task4', 'task5', 'task6']
        # Associate the task name with a state machine
        plans = {'task1': task1_2.Task1,
                 'task2': task1_2.Task1,
                 'task3': task4.Task4,
                 'task4': task4.Task4,
                 'task5': task4.Task4,
                 'task6': task6.Task6}

        # Associate the task name with the parameter for the state machine
        plan_args = {'task1': [False, 'task1'],
                     'task2': [False, 'task2'],
                     'task3': ['task3'],
                     'task4': ['task4'],
                     'task5': ['task5'],
                     'task6': ['task6']}

        with self:
            task_success = 'success'

            if init_sim:
                smach.StateMachine.add('InitSimulation', InitSimulation(task_name),
                                       transitions={'task1': 'task1',
                                                    'task2': 'task2',
                                                    'task3': 'task3',
                                                    'task4': 'task4',
                                                    'task5': 'task5',
                                                    'task6': 'task6',
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(self.savelog),
                                       transitions={'success': 'StopNodes',
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopNodes', start_nodes.StopNodes(),
                                       transitions={'success': 'success',
                                                    'fail': 'fail'})

                task_success = 'StopSimulation'
            else:
                smach.StateMachine.add('GetYaml', GetYaml(),
                                       transitions={'success': task_name,
                                                    'fail': 'fail'})

            for task in available_tasks:
                smach.StateMachine.add(task, plans[task](*plan_args[task]),
                                       transitions={'success': task_success,
                                                    'fail': 'fail'})


class InitSimulation(smach.StateMachine):
    def __init__(self, task_name):
        smach.StateMachine.__init__(self, outcomes=['task1', 'task2', 'task3', 'task4', 'task5', 'task6', 'fail'],
                                    input_keys=['initialization_time', 'logging'],
                                    output_keys=['objects_found', 'yaml', 'perception_process', 'manipulation_process',
                                                 'manipulation_conveyor_frames_process', 'classifier_process',
                                                 'perception_logger_process', 'manipulation_logger_process',
                                                 'manipulation_conveyor_frames_logger_process',
                                                 'classifier_logger_process'])
        with self:
            smach.StateMachine.add('StartSimulation', start_nodes.StartSimulation(task_name),
                                   transitions={'success': 'StartManipulation',
                                                'fail': 'StartSimulation'})
            smach.StateMachine.add('StartManipulation', start_nodes.StartManipulation(),
                                   transitions={'success': 'StartPerception',
                                                'fail': 'StartManipulation'})
            smach.StateMachine.add('StartPerception', start_nodes.StartPerception(),
                                   transitions={'success': 'StartClassifier',
                                                'fail': 'StartPerception'})
            smach.StateMachine.add('StartClassifier', start_nodes.StartClassifier(),
                                   transitions={'success': 'DetermineTaskType',
                                                'fail': 'StartClassifier'})
            smach.StateMachine.add('DetermineTaskType', DetermineTaskType(),
                                   transitions={'task1': 'task1',
                                                'task2': 'task2',
                                                'task3': 'task3',
                                                'task4': 'task4',
                                                'task5': 'task5',
                                                'task6': 'task6',
                                                'fail': 'fail'})


class DetermineTaskType(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['task1', 'task2', 'task3', 'task4', 'task5', 'task6', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, ud):
        rospy.loginfo('Executing state DetermineTaskType')
        task_type = ud.yaml.task_type
        if task_type == Task.TASK_1:
            ret = 'task1'
        elif task_type == Task.TASK_2:
            ret = 'task2'
        elif task_type == Task.TASK_3:
            ret = 'task3'
        elif task_type == Task.TASK_4:
            ret = 'task4'
        elif task_type == Task.TASK_5:
            ret = 'task5'
        elif task_type == Task.TASK_6:
            ret = 'task6'
        else:
            ret = 'fail'
        rospy.loginfo('Executing task is from type ' + str(ret) + '.')
        return ret


class GetYaml(smach.State):
    _yaml = None

    _lock = None

    def __init__(self):
        self._lock = threading.Lock()
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['yaml'])

    def execute(self, userdata):
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.parse_yaml)
        rospy.loginfo('Waiting for yaml')

        self._lock.acquire()
        while self._yaml is None:
            self._lock.release()
            time.sleep(0.05)
            self._lock.acquire()

        rospy.loginfo('Got yaml %s'%str(self._yaml))
        userdata.yaml = self._yaml
        self._lock.release()

        return 'success'

    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: %s'%str(self._yaml))
        self._lock.release()

