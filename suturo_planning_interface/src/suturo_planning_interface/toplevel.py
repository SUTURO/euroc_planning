import traceback
import os
import sys
import rospy
import threading
import subprocess
import time
import start_nodes
import signal
import atexit
from suturo_msgs.msg import Task
import utils

__handling_exit = False
# TODO: start classes when starting node

def exit_handler(signum=None, frame=None):
    print('toplevel: exit_handler')
    global __handling_exit
    if __handling_exit:
        print('toplevel: Already handling exit.')
        return
    __handling_exit = True
    print('toplevel: Exiting exit_handler.')

signal.signal(signal.SIGTERM, exit_handler)
signal.signal(signal.SIGINT, exit_handler)
atexit.register(exit_handler)


def handle_uncaught_exception(e, initialization_time, logging, parent_pid):
    print('Uncaught exception: ' + str(e))
    if not start_nodes.executed_test_node_check:
        start_nodes.check_node(initialization_time, logging)
    print('Terminating task.')
    if parent_pid is not None:
        print('Sending signal to parent process.')
        os.kill(parent_pid, signal.SIGUSR1)
    else:
        print('Unknown parent. Won\'t send signal.')
    rospy.signal_shutdown('Terminating Task due to unhandled exception.')


# TODO: toplevel Parameter als Dict oder Klassenvariablen oder als Parameter direkt verwenden
class Toplevel(object):
    def __init__(self, init_sim, task_name, savelog, initialization_time, logging, parent_pid=None):
        self.task_name = task_name
        self.init_sim = init_sim

        print('toplevel: logging: ' + str(logging))
        if logging == 1:
            print('toplevel: Logging planning to console.')
        else:
            print('toplevel: Logging planning to files.')
            __logger_process = utils.start_logger(subprocess.PIPE, initialization_time, 'Planning', logging)
            sys.stderr = __logger_process.stdin
            sys.stdout = __logger_process.stdin

    # TODO:replace sleep with a better solution
    def init_simulation(self, task_name, userdata):
        start_nodes.StartSimulation(task_name)
        time.sleep(5)
        start_nodes.StartManipulation()
        time.sleep(5)
        start_nodes.StartPerception()
        time.sleep(5)
        start_nodes.StartClassifier()
        time.sleep(5)
        # TaskTypeDeterminer.determine_task_type(userdata)

# TODO:Exceptionhandling anpassen
    def start_task(self):
        toplevel_thread = threading.Thread(target=self.execute_task)
        rospy.loginfo('toplevel: Starting smach thread.')
        toplevel_thread.start()
        rospy.loginfo('toplevel: Waiting for smach thread to terminate.')
        # Wait for ctrl-c
        rospy.spin()

    def execute_task(self):
            try:
                if self.init_sim:
                    self.process_with_init_parameter()
                else:
                    self.process_with_plan_parameter()
            except BaseException, e:
                print('BaseException while executing Task:')
                print(traceback.print_exc())
                # handle_uncaught_exception(sys.exc_info()[0], initialization_time, logging, parent_pid)
            except:
                print('Unhandled Exception while executing Task:')
                print(traceback.print_exc())
                # handle_uncaught_exception(sys.exc_info()[0], initialization_time, logging, parent_pid)

    def process_with_init_parameter(self):
        self.init_simulation(self.task_name)

    def process_with_plan_parameter(self):
        YamlHandler.get_yaml()
        TaskTypeDeterminer.determine_task_type(self.userdata)



class TaskTypeDeterminer(object):
    @staticmethod
    def determine_task_type(userdata):
        rospy.loginfo('Executing state DetermineTaskType')
        task_type = userdata.yaml.task_type
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


class YamlHandler(object):
    @staticmethod
    def get_yaml(self, userdata):
        self._yaml = None
        self._lock = None

        self._lock = threading.Lock()
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.parse_yaml)
        rospy.loginfo('Waiting for yaml')

        self._lock.acquire()
        while self._yaml is None:
            self._lock.release()
            time.sleep(0.05)
            self._lock.acquire()

        rospy.loginfo('Got yaml %s' % str(self._yaml))
        userdata.yaml = self._yaml
        self._lock.release()

    @staticmethod
    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: %s' % str(self._yaml))
        self._lock.release()