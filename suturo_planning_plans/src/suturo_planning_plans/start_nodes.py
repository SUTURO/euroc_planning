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


perception_process = None
manipulation_process = None
classifier_process = None
perception_logger_process = None
manipulation_logger_process = None
classifier_logger_process = None
executed_test_node_check = False
__handling_exit = False


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging'],
                             output_keys=['manipulation_process', 'manipulation_logger_process'])
        self.__move_group_status = False
        self.__move_group_status_subscriber = None

    def wait_for_move_group_status(self, msg):
        self.__move_group_status_subscriber.unregister()
        rospy.loginfo('/move_group/status has been published.')
        self.__move_group_status = True

    def execute(self, userdata):
        rospy.loginfo('Executing TestNode init.')
        subprocess.Popen('rosrun euroc_launch TestNode --init', shell=True)
        rospy.loginfo('Executing state StartManipulation')
        global manipulation_process
        global manipulation_logger_process
        manipulation_process, manipulation_logger_process =\
            utils.start_node('roslaunch euroc_launch manipulation.launch', userdata.initialization_time,
                             'Manipulation', userdata.logging)
        userdata.manipulation_process = manipulation_process
        userdata.manipulation_logger_process = manipulation_logger_process
        rospy.loginfo('Waiting for /move_group/status topic.')
        self.__move_group_status_subscriber = rospy.Subscriber('/move_group/status', GoalStatusArray,
                                                               self.wait_for_move_group_status)
        while not self.__move_group_status:
            time.sleep(1)
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging'],
                             output_keys=['perception_process', 'perception_logger_process'])
        self.__subscriber = None
        self.__perception_ready = False
        self.__required_nodes = None
        self.__started_nodes = []

    def wait_for_perception(self, msg):
        rospy.loginfo('Executing wait_for_perception')
        if len(msg.required_nodes) > 0 and self.__required_nodes is None:
            rospy.loginfo('required_nodes: ' + str(msg.required_nodes))
            self.__required_nodes = msg.required_nodes
        elif self.__required_nodes is not None and msg.started_node in self.__required_nodes:
            rospy.loginfo('Started node: ' + str(msg.started_node))
            self.__started_nodes.append(msg.started_node)
        else:
            rospy.loginfo('Unhandled Message:\n' + str(msg))

        if self.__required_nodes is not None:
            for n in self.__required_nodes:
                if n not in self.__started_nodes:
                    rospy.loginfo('Node ' + str(n) + ' has not been started yet. Still waiting.')
                    return
            rospy.loginfo('All nodes have been started.')
            if True:
                self.__perception_ready = True
                try:
                    self.__subscriber.unregister()
                except AssertionError, e:
                    rospy.loginfo('Accidentally tried to unregister already unregistered subscriber.')
                    rospy.loginfo(e)
        else:
            rospy.loginfo('Still waiting for perception.')

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        rospy.loginfo('Subscribing to /suturo/perception_node_status.')
        self.__subscriber = rospy.Subscriber('/suturo/perception_node_status', PerceptionNodeStatus,
                                             self.wait_for_perception)
        global perception_process
        global perception_logger_process
        perception_process, perception_logger_process =\
            utils.start_node('roslaunch euroc_launch perception_task1.launch', userdata.initialization_time,
                             'Perception', userdata.logging)
        userdata.perception_process = perception_process
        userdata.perception_logger_process = perception_logger_process
        while not self.__perception_ready:
            time.sleep(1)
        return 'success'


class StartPerceptionTask6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time'],
                             output_keys=['perception_process', 'perception_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerceptionTask6')
        global perception_process
        global perception_logger_process
        perception_process, perception_logger_process =\
            utils.start_node('roslaunch euroc_launch perception_task6.launch', userdata.initialization_time,
                             'Perception', userdata.logging)
        userdata.perception_process = perception_process
        userdata.perception_logger_process = perception_logger_process
        return 'success'


class StartClassifier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'logging'],
                             output_keys=['classifier_process', 'classifier_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartClassifier')
        global classifier_process
        global classifier_logger_process
        classifier_process, classifier_logger_process =\
            utils.start_node('rosrun suturo_perception_classifier classifier.py', userdata.initialization_time,
                             'Classifier', userdata.logging)
        userdata.classifier_process = classifier_process
        userdata.classifier_logger_process = classifier_logger_process
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
        check_node(userdata.initialization_time, userdata.logging)
        if self.savelog:
            save_task()
        stop_task()
        time.sleep(3)
        return 'success'


def check_node(initialization_time, logging):
    global executed_test_node_check
    rospy.loginfo('Executing TestNode check.')
    test_node, test_node_logger = utils.start_node('rosrun euroc_launch TestNode --check', initialization_time,
                                                   'TestNode check', logging)
    rospy.loginfo('Waiting for TestNode check to terminate. PID: ' + str(test_node.pid))
    if not utils.wait_for_process(test_node, 15):
        rospy.loginfo('Killing TestNode check.')
        exterminate(test_node.pid, signal.SIGKILL)
    #test_node.wait()
    if test_node_logger is not None:
        rospy.loginfo('Killing TestNode logger with pid ' + str(test_node_logger.pid) + '.')
        exterminate(test_node_logger.pid, signal.SIGINT)
    executed_test_node_check = True
    rospy.loginfo('Finished TestNode check.')


class StopNodes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['perception_process', 'manipulation_process', 'classifier_process',
                                         'perception_logger_process', 'manipulation_logger_process',
                                         'classifier_logger_process'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Stopping Nodes')
        if userdata.perception_process is not None:
            exterminate(userdata.perception_process.pid, signal.SIGKILL)
        if userdata.manipulation_process is not None:
            exterminate(userdata.manipulation_process.pid, signal.SIGKILL)
        if userdata.classifier_process is not None:
            exterminate(userdata.classifier_process.pid, signal.SIGKILL)
        if userdata.perception_logger_process is not None:
            exterminate(userdata.perception_logger_process.pid, signal.SIGINT)
        if userdata.manipulation_logger_process is not None:
            exterminate(userdata.manipulation_logger_process.pid, signal.SIGINT)
        if userdata.classifier_logger_process is not None:
            exterminate(userdata.classifier_logger_process.pid, signal.SIGINT)
        rospy.signal_shutdown('Finished plan. Shutting down Node.')
        time.sleep(3)
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
        exterminate(manipulation_process.pid, signal.SIGINT)
    global perception_process
    if perception_process is not None:
        print 'Killing perception'
        exterminate(perception_process.pid, signal.SIGINT)
    global classifier_process
    if classifier_process is not None:
        print 'Killing classifier'
        exterminate(classifier_process.pid, signal.SIGINT)
    global manipulation_logger_process
    if manipulation_logger_process is not None:
        print('Killing manipulation logger with pid ' + str(manipulation_logger_process.pid) + '.')
        exterminate(manipulation_logger_process.pid, signal.SIGINT)
    global perception_logger_process
    if perception_logger_process is not None:
        print('Killing perception logger with pid ' + str(perception_logger_process.pid) + '.')
        exterminate(perception_logger_process.pid, signal.SIGINT)
    global classifier_logger_process
    if classifier_logger_process is not None:
        print('Killing classifier logger with pid' + str(classifier_logger_process.pid) + '.')
        exterminate(classifier_logger_process.pid, signal.SIGINT)
    print('start_nodes: Exiting exit_handler')


atexit.register(exit_handler)
signal.signal(signal.SIGINT,  exit_handler)