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
from suturo_planning_plans import utils
from suturo_planning_task_selector import save_task
from suturo_planning_task_selector import stop_task
from rosgraph_msgs.msg import Clock
from actionlib_msgs.msg import GoalStatusArray
from suturo_perception_msgs.msg import PerceptionNodeStatus
from suturo_manipulation_msgs.msg import ManipulationNodeStatus
from suturo_msgs.msg import Task
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r
from suturo_planning_task_selector import task_selector


perception_process = None
manipulation_process = None
manipulation_conveyor_frames_process = None
classifier_process = None
executed_test_node_check = False
__handling_exit = False


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging', 'yaml'],
                             output_keys=['manipulation_process', 'manipulation_conveyor_frames_process'])
        self.__move_group_status = False
        self.__move_group_status_subscriber = None
        self.__nodes_subscriber = None
        self.__manipulation_nodes_ready = False
        self.__required_nodes = None
        self.__started_nodes = []

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

    def execute(self, userdata):
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
            utils.start_node('roslaunch euroc_launch manipulation.launch', userdata.initialization_time,
                             userdata.logging, 'Manipulation')
        userdata.manipulation_process = manipulation_process

        while not self.__move_group_status and not self.__manipulation_nodes_ready:
            time.sleep(1)

        task_type = userdata.yaml.task_type
        if task_type == Task.TASK_6:
            global manipulation_conveyor_frames_process
            rospy.loginfo('Starting publish_conveyor_frames.')
            manipulation_conveyor_frames_process, manipulation_conveyor_frames_logger_process =\
                utils.start_node('rosrun suturo_planning_manipulation publish_conveyor_frames.py',
                                 userdata.initialization_time, userdata.logging, 'Conveyor frames')
            userdata.manipulation_conveyor_frames_process = manipulation_conveyor_frames_process
        else:
            userdata.manipulation_conveyor_frames_process = None
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging', 'yaml'],
                             output_keys=['perception_process'])
        self.__subscriber = None
        self.__perception_ready = False
        self.__required_nodes = None
        self.__started_nodes = []

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

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        rospy.loginfo('Subscribing to /suturo/perception_node_status.')
        self.__subscriber = rospy.Subscriber('/suturo/perception_node_status', PerceptionNodeStatus,
                                             self.wait_for_perception)
        rospy.sleep(1)
        task_type = userdata.yaml.task_type
        if task_type == Task.TASK_4:
            task_num = '4'
        elif task_type == Task.TASK_6:
            task_num = '6'
        else:
            task_num = '1'
        global perception_process
        perception_process, perception_logger_process =\
            utils.start_node('roslaunch euroc_launch perception_task' + task_num + '.launch', userdata.initialization_time,
                             userdata.logging, 'Perception')
        userdata.perception_process = perception_process
        while not self.__perception_ready:
            time.sleep(1)
        return 'success'


class StartClassifier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging'],
                             output_keys=['classifier_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartClassifier')
        global classifier_process
        classifier_process, classifier_logger_process =\
            utils.start_node('rosrun suturo_perception_classifier classifier.py', userdata.initialization_time,
                             userdata.logging, 'Classifier')
        userdata.classifier_process = classifier_process
        return 'success'


class StartSimulation(smach.State):
    task_name = ''

    def __init__(self, task_name):
        self.task_name = task_name
        self.new_clock = False
        self.clock = None
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['yaml', 'objects_found'])

    def wait_for_clock(self, msg):
        rospy.loginfo('Clock has been published.')
        self.clock.unregister()
        self.new_clock = True

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        userdata.yaml = suturo_planning_task_selector.start_task(self.task_name)
        rospy.loginfo('Got YAML description')
        userdata.objects_found = []
        rospy.loginfo('Waiting for clock.')
        self.clock = rospy.Subscriber('clock', Clock, self.wait_for_clock)
        while not self.new_clock:
            time.sleep(1)
        return 'success'


class StopSimulation(smach.State):
    def __init__(self, savelog):
        self.savelog = savelog
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'initialization_time', 'logging'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state StopSimulation')
        if not executed_test_node_check:
            check_node(userdata.initialization_time, userdata.logging)
        if self.savelog:
            save_task()
        stop_task()
        time.sleep(3)
        rospy.loginfo('Finished state StopSimulation')
        return 'success'


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


class StopNodes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['perception_process', 'manipulation_process',
                                         'manipulation_conveyor_frames_process', 'classifier_process'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state StopNodes.')
        if userdata.perception_process is not None:
            exterminate(userdata.perception_process.pid, signal.SIGKILL, r=True)
        if userdata.manipulation_process is not None:
            exterminate(userdata.manipulation_process.pid, signal.SIGKILL, r=True)
        if userdata.manipulation_conveyor_frames_process is not None:
            exterminate(userdata.manipulation_conveyor_frames_process.pid, signal.SIGKILL, r=True)
        if userdata.classifier_process is not None:
            exterminate(userdata.classifier_process.pid, signal.SIGKILL, r=True)
        rospy.signal_shutdown('Finished plan. Shutting down Node.')
        time.sleep(3)
        rospy.loginfo('Finished state StopNodes.')
        return 'success'


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
    print('start_nodes: Exiting exit_handler')


atexit.register(exit_handler)
signal.signal(signal.SIGINT,  exit_handler)