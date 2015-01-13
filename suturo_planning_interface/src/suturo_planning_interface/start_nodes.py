import sys
import rospy
import smach
import time
import subprocess
import os
import signal
import atexit
from suturo_planning_manipulation.total_annihilation import exterminate
import suturo_planning_task_selector
import threading
from suturo_planning_interface import utils
from suturo_planning_task_selector import save_task
from suturo_planning_task_selector import stop_task
from rosgraph_msgs.msg import Clock
from actionlib_msgs.msg import GoalStatusArray
from suturo_perception_msgs.msg import PerceptionNodeStatus
from suturo_manipulation_msgs.msg import ManipulationNodeStatus
from suturo_msgs.msg import Task
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r

perception_process = None
manipulation_process = None
manipulation_conveyor_frames_process = None
classifier_process = None
executed_test_node_check = False
__handling_exit = False


class StartYamlParser():
    def __init__(self):
        self.start_service()

    def start_service(self):
        self.start_yaml_parser = rospy.Service('suturo/state/start_yaml_parser', TaskDataService,
                                                        self.start_yaml_parser)

    def start_yaml_parser(self, req):
        self.yaml_parser = YamlPars0r()


class StartManipulation(object):
    def __init__(self):
        self.__move_group_status = False
        self.__move_group_status_subscriber = None
        self.__nodes_subscriber = None
        self.__manipulation_nodes_ready = False
        self.__required_nodes = None
        self.__started_nodes = []
        self.start_service()

    def start_service(self):
        self.start_manipulation_service = rospy.Service('suturo/state/start_manipulation', TaskDataService,
                                                        self.start_manipulation)

    def start_manipulation(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing TestNode init.')
        subprocess.Popen('rosrun euroc_launch TestNode --init', shell=True)
        rospy.loginfo('Executing state StartManipulation')
        rospy.loginfo('Subscribing to /suturo/manipulation_node_status.')
        self.__nodes_subscriber = rospy.Subscriber('/suturo/manipulation_node_status', ManipulationNodeStatus,
                                             self.wait_for_manipulation)
        rospy.loginfo('Subscribign to /move_group/status topic.')
        rospy.sleep(1)
        self.__move_group_status_subscriber = rospy.Subscriber('/move_group/status', GoalStatusArray,
                                                               self.wait_for_move_group_status)
        global manipulation_process
        manipulation_process, manipulation_logger_process =\
            utils.start_node('roslaunch euroc_launch manipulation.launch', resp.taskdata.initialization_time,
                             resp.taskdata.logging, 'Manipulation')

        while not self.__move_group_status and not self.__manipulation_nodes_ready:
            time.sleep(1)

        task_type = resp.taskdata.yaml.task_type
        if task_type == Task.TASK_6:
            global manipulation_conveyor_frames_process
            rospy.loginfo('Starting publish_conveyor_frames.')
            manipulation_conveyor_frames_process, manipulation_conveyor_frames_logger_process =\
                utils.start_node('rosrun suturo_planning_manipulation publish_conveyor_frames.py',
                                 resp.taskdata.initialization_time, resp.taskdata.logging, 'Conveyor frames')
            manipulation_conveyor_frames_process = manipulation_conveyor_frames_process
        else:
            manipulation_conveyor_frames_process = None
        rospy.loginfo('Sucessfully started Manipulation')
        resp.result = 'success'
        return resp

    def wait_for_manipulation(self, msg):
        rospy.loginfo('Executing wait_for_manipulation')
        if len(msg.required_nodes) > 0 and \
                msg.started_node == ManipulationNodeStatus.REQUIRED_NODES_INCOMING and \
                self.__required_nodes is None:

            rospy.loginfo('man: required_nodes: ' + str(msg.required_nodes))
            self.__required_nodes = msg.required_nodes
        elif self.__required_nodes is not None and msg.started_node in self.__required_nodes:
            rospy.loginfo('man: Started node: ' + str(msg.started_node))
            self.__started_nodes.append(msg.started_node)
        else:
            rospy.loginfo('man: Unhandled Message:\n' + str(msg))

        if self.__required_nodes is not None:
            for n in self.__required_nodes:
                if n not in self.__started_nodes:
                    rospy.loginfo('man: Node ' + str(n) + ' has not been started yet. Still waiting.')
                    return
            rospy.loginfo('man: All nodes have been started.')
            self.__manipulation_nodes_ready = True
            try:
                self.__nodes_subscriber.unregister()
            except AssertionError, e:
                rospy.loginfo('man: Accidentally tried to unregister already unregistered subscriber.')
                rospy.loginfo(e)
        else:
            rospy.loginfo('Still waiting for manipulation.')

    def wait_for_move_group_status(self, msg):
        self.__move_group_status_subscriber.unregister()
        rospy.loginfo('/move_group/status has been published.')
        self.__move_group_status = True



class StartPerception(object):
    def __init__(self):
        self.__subscriber = None
        self.__perception_ready = False
        self.__required_nodes = None
        self.__started_nodes = []
        self.task_type_service = None
        self.start_service()

    def start_service(self):
        self.start_perception_service = rospy.Service('suturo/state/start_perception', TaskDataService,
                                                      self.start_perception)

    def wait_for_perception(self, msg):
        rospy.loginfo('per: Executing wait_for_perception')
        if len(msg.required_nodes) > 0 and \
                msg.started_node == PerceptionNodeStatus.REQUIRED_NODES_INCOMING and \
                self.__required_nodes is None:

            rospy.loginfo('per: required_nodes: ' + str(msg.required_nodes))
            self.__required_nodes = msg.required_nodes
        elif self.__required_nodes is not None and msg.started_node in self.__required_nodes:
            rospy.loginfo('per: Started node: ' + str(msg.started_node))
            self.__started_nodes.append(msg.started_node)
        else:
            rospy.loginfo('per: Unhandled Message:\n' + str(msg))

        if self.__required_nodes is not None:
            for n in self.__required_nodes:
                if n not in self.__started_nodes:
                    rospy.loginfo('per: Node ' + str(n) + ' has not been started yet. Still waiting.')
                    return
            rospy.loginfo('per: All nodes have been started.')
            self.__perception_ready = True
            try:
                self.__subscriber.unregister()
            except AssertionError, e:
                rospy.loginfo('per: Accidentally tried to unregister already unregistered subscriber.')
                rospy.loginfo(e)
        else:
            rospy.loginfo('per: Still waiting for perception.')


    def start_perception(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state StartPerception')
        rospy.loginfo('Subscribing to /suturo/perception_node_status.')
        self.__subscriber = rospy.Subscriber('/suturo/perception_node_status', PerceptionNodeStatus,
                                             self.wait_for_perception)
        rospy.sleep(1)
        task_type = resp.taskdata.yaml.task_type
        if task_type == Task.TASK_4:
            task_num = '4'
        elif task_type == Task.TASK_6:
            task_num = '6'
        else:
            task_num = '1'
        global perception_process
        perception_process, perception_logger_process =\
            utils.start_node('roslaunch euroc_launch perception_task' + task_num + '.launch',
                             resp.taskdata.initialization_time,
                             resp.taskdata.logging, 'Perception')
        while not self.__perception_ready:
            time.sleep(1)
        rospy.loginfo('Sucessfully started Perception')
        resp.result = 'success'
        return resp


class StartClassifier(object):
    def __init__(self):
        self.start_service()

    def start_service(self):
        self.start_classifier_service = rospy.Service('suturo/state/start_classifier', TaskDataService,
                                                      self.start_classifier)

    def start_classifier(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state StartClassifier')
        global classifier_process
        classifier_process, classifier_logger_process =\
            utils.start_node('rosrun suturo_perception_classifier classifier.py', req.taskdata.initialization_time,
                             req.taskdata.logging, 'Classifier')
        resp.result = 'success'
        rospy.loginfo('Sucessfully started Classifier')
        return resp


class StartSimulation(object):
    task_name = ''

    def __init__(self):
        self.new_clock = False
        self.clock = None
        self.start_service()

    def start_service(self):
        self.start_simulation_service = rospy.Service('suturo/state/start_simulation', TaskDataService,
                                                      self.start_simulation)

    def start_simulation(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state StartSimulation')
        resp.taskdata.yaml = suturo_planning_task_selector.start_task(resp.taskdata.name)
        rospy.loginfo('Got YAML description')
        resp.taskdata.objects_found = []
        rospy.loginfo('Waiting for clock.')
        self.clock = rospy.Subscriber('clock', Clock, self.wait_for_clock)
        while not self.new_clock:
            time.sleep(1)
        rospy.loginfo('Sucessfully started Simulation')
        resp.result = 'success'
        return resp

    def wait_for_clock(self, msg):
        rospy.loginfo('Clock has been published.')
        self.clock.unregister()
        self.new_clock = True


class StopSimulation(object):

    def __init__(self, savelog):
        """
        :param savelog: boolean, whether logs should be saved
        :return:
        """
        self.savelog = savelog
        self.start_service()

    def start_service(self):
        self.stop_simulation_service = rospy.Service('suturo/state/stop_simulation', TaskDataService,
                                                     self.stop_simulation)

    def stop_simulation(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state StopSimulation')
        if not executed_test_node_check:
            check_node(resp.taskdata.initialization_time, resp.taskdata.logging)
        if self.savelog:
            save_task()
        stop_task()
        time.sleep(3)
        rospy.loginfo('Finished state StopSimulation')
        resp.result = 'success'
        return resp


def check_node(initialization_time, logging):
    global executed_test_node_check
    rospy.loginfo('Executing TestNode check.')
    rospy.loginfo('rospy.is_shutdown(): ' + str(rospy.is_shutdown()))
    test_node, test_node_logger = utils.start_node('rosrun euroc_launch TestNode --check', initialization_time,
                                                   logging, 'TestNode check')
    if not utils.wait_for_process(test_node, 15):
        rospy.loginfo('Killing TestNode check.')
        exterminate(test_node.pid, signal.SIGKILL)
    executed_test_node_check = True
    rospy.loginfo('Finished TestNode check.')


class StopNodes(object):
    def __init__(self):
        self.start_service()

    def start_service(self):
        self.stop_nodes_service = rospy.Service('suturo/state/stop_nodes', TaskDataService, self.stop_nodes)

    def stop_nodes(self, req):
        resp = TaskDataServiceResponse()
        resp.taskdata = req.taskdata
        rospy.loginfo('Executing state StopNodes.')
        if perception_process is not None:
            exterminate(perception_process.pid, signal.SIGKILL, r=True)
        if manipulation_process is not None:
            exterminate(manipulation_process.pid, signal.SIGKILL, r=True)
        if manipulation_conveyor_frames_process is not None:
            exterminate(manipulation_conveyor_frames_process.pid, signal.SIGKILL, r=True)
        if classifier_process is not None:
            exterminate(classifier_process.pid, signal.SIGKILL, r=True)
        rospy.signal_shutdown('Finished plan. Shutting down Node.')
        time.sleep(3)
        rospy.loginfo('Finished state StopNodes.')
        resp.result = 'success'
        return resp


def exit_handler(signum=None, frame=None):
    print('start_nodes: exit_handler')
    global __handling_exit
    if __handling_exit:
        print('start_nodes: Already handling exit.')
        return
    __handling_exit = True
    global manipulation_process
    if manipulation_process is not None:
        print 'Killing manipulation'
        exterminate(manipulation_process.pid, signal.SIGKILL, r=True)
    global manipulation_conveyor_frames_process
    if manipulation_conveyor_frames_process is not None:
        print('Killing manipulation_conveyor_frames_process.')
        exterminate(manipulation_conveyor_frames_process.pid, signal.SIGKILL, r=True)
    global perception_process
    if perception_process is not None:
        print 'Killing perception'
        exterminate(perception_process.pid, signal.SIGKILL, r=True)
    global classifier_process
    if classifier_process is not None:
        print 'Killing classifier'
        exterminate(classifier_process.pid, signal.SIGKILL, r=True)
    time.sleep(3)
    rospy.signal_shutdown('Finished handling exit. Shutting down Node.')
    print('start_nodes: Exiting exit_handler')


# atexit.register(exit_handler)
signal.signal(signal.SIGINT,  exit_handler)