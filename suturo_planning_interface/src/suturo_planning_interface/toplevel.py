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
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse
from search_objects import SearchObjects
from scan_map import MapScanner
from scan_obstacles import ScanObstacles
from classify_objects import ClassifyObjects
from focus_objects import FocusObjects
from pose_estimate_objects import PoseEstimateObject
from scan_shadow import ScanShadow
from start_nodes import StartClassifier, StartManipulation, StartPerception, StartSimulation, StopNodes, StopSimulation
from suturo_planning_interface import utils
from suturo_planning_manipulation.manipulation import Manipulation

import utils

_pro_task_selector = None
_save_log = False
__handling_exit = False

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


class Toplevel(object):
    def __init__(self, initialization_time, logging):
        self.configure_logging(logging, initialization_time)
        rospy.init_node('suturo/toplevel')
        self.start_init_service()

    def configure_logging(self, logging, initialization_time):
        print('toplevel: logging: ' + str(logging))
        if logging == 1:
            print('toplevel: Logging planning to console.')
        else:
            print('toplevel: Logging planning to files.')
            __logger_process = utils.start_logger(subprocess.PIPE, initialization_time, 'Planning', logging)
            sys.stderr = __logger_process.stdin
            sys.stdout = __logger_process.stdin

    def start_init_service(self):
        self.init_service = rospy.Service('suturo/toplevel/init', TaskDataService, self.init)
        rospy.spin()

    def init(self, req):
        self.create_manipulation()
        self.start_state_nodes()
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        resp.result = 'success'
        return resp

    def create_manipulation(self):
        utils.manipulation = Manipulation()

    def start_state_nodes(self):
        self.search_object_state = SearchObjects()
        self.determine_task_type_state = TaskTypeDeterminer()
        self.map_scanner_state = MapScanner()
        self.scan_obstacles_state = ScanObstacles()
        self.classify_objects_state = ClassifyObjects()
        self.focus_objects_state = FocusObjects()
        # TODO: Pose estimate object(s) Name anpassen
        self.pose_estimate_objects_state = PoseEstimateObject()
        self.scan_map_state = MapScanner()
        self.scan_shadow_state = ScanShadow()
        self.start_simulation_state = StartSimulation()
        self.start_perception_state = StartPerception()
        self.start_manipulation_sate = StartManipulation()
        self.start_classifier_state = StartClassifier()
        self.stop_simulation_state = StopSimulation()
        self.stop_nodes_state = StopNodes()


class YamlHandler(object):
    def __init__(self):
        self.start_service()
        self._yaml = None
        self._lock = None

    def start_service(self):
        self.yaml_handler_service = rospy.Service('suturo/state/YamlHandler', TaskDataService, self.get_yaml)

    def get_yaml(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        self._lock = threading.Lock()
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.parse_yaml)
        rospy.loginfo('Waiting for yaml')

        self._lock.acquire()
        while self._yaml is None:
            self._lock.release()
            time.sleep(0.05)
            self._lock.acquire()

        rospy.loginfo('Got yaml %s' % str(self._yaml))
        self._lock.release()

        resp.taskdata.yaml = self._yaml
        resp.result = 'success'
        return resp

    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: %s' % str(self._yaml))
        self._lock.release()


class TaskTypeDeterminer(object):
    def __init__(self):
        self.start_service()

    def start_service(self):
        self.task_type_service = rospy.Service('suturo/state/TaskTypeDeterminer', TaskDataService, self.determine_task_type)

    def determine_task_type(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state DetermineTaskType')
        task_type = resp.taskdata.yaml.task_type
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

        resp.result = ret
        rospy.loginfo('Executing task is from type ' + str(ret) + '.')
        return resp