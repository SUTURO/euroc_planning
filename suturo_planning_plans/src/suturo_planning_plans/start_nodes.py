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


perception_process = 0
manipulation_process = 0
classifier_process = 0
perception_logger_process = 0
manipulation_logger_process = 0
classifier_logger_process = 0


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'log_to_console_only'],
                             output_keys=['manipulation_process', 'manipulation_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing TestNode init.')
        subprocess.Popen('rosrun euroc_launch TestNode --init', shell=True)
        rospy.loginfo('Executing state StartManipulation')
        global manipulation_process
        global manipulation_logger_process
        manipulation_process, manipulation_logger_process =\
            utils.start_node('roslaunch euroc_launch manipulation.launch', userdata.initialization_time,
                             'Manipulation', userdata.log_to_console_only)
        userdata.manipulation_process = manipulation_process
        userdata.manipulation_logger_process = manipulation_logger_process
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'log_to_console_only'],
                             output_keys=['perception_process', 'perception_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        global perception_process
        global perception_logger_process
        perception_process, perception_logger_process =\
            utils.start_node('roslaunch euroc_launch perception_task1.launch', userdata.initialization_time,
                             'Perception', userdata.log_to_console_only)
        userdata.perception_process = perception_process
        userdata.perception_logger_process = perception_logger_process
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
                             'Perception', userdata.log_to_console_only)
        userdata.perception_process = perception_process
        userdata.perception_logger_process = perception_logger_process
        return 'success'


class StartClassifier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time', 'log_to_console_only'],
                             output_keys=['classifier_process', 'classifier_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartClassifier')
        global classifier_process
        global classifier_logger_process
        classifier_process, classifier_logger_process =\
            utils.start_node('rosrun suturo_perception_classifier classifier.py', userdata.initialization_time,
                             'Classifier', userdata.log_to_console_only)
        userdata.classifier_process = classifier_process
        userdata.classifier_logger_process = classifier_logger_process
        return 'success'


class StartSimulation(smach.State):
    task_name = ''

    def __init__(self, task_name):
        self.task_name = task_name
        self.new_clock = False
        self.lock = threading.Lock()
        self.clock = None
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['yaml', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        userdata.yaml = suturo_planning_task_selector.start_task(self.task_name)
        rospy.loginfo('Got YAML description')
        userdata.objects_found = []
        rospy.loginfo('Waiting for clock.')
        self.clock = rospy.Subscriber('clock', Clock, self.wait_for_clock())
        self.lock.acquire()
        while not self.new_clock:
            self.lock.release()
            rospy.sleep(1)
            self.lock.acquire()
        self.lock.release()
        self.clock.unregister()
        return 'success'

    def wait_for_clock(self):
        rospy.loginfo('Clock has been published.')
        self.lock.acquire()
        self.new_clock = True
        self.lock.release()


class StopSimulation(smach.State):
    def __init__(self, savelog):
        self.savelog = savelog
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'initialization_time', 'log_to_console_only'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state StopSimulation')
        check_node(userdata.initialization_time, userdata.log_to_console_only)
        if self.savelog:
            save_task()
        stop_task()
        time.sleep(3)
        return 'success'


def check_node(initialization_time, log_to_console_only):
    rospy.loginfo('Executing TestNode check.')

    test_node, test_node_logger = utils.start_node('rosrun euroc_launch TestNode --check', initialization_time,
                                                   'TestNode check', log_to_console_only)
    test_node.wait()
    if test_node_logger != 0:
        rospy.loginfo('Killing TestNode logger with pid ' + str(test_node_logger.pid) + '.')
        test_node_logger.terminate()
        #os.killpg(os.getpgid(test_node_logger.pid), signal.SIGTERM)
        time.sleep(1)
        if test_node_logger.poll() is not None:
            rospy.loginfo('Test node logger is still running. Sending SIGINT.')
            try:
                #os.killpg(os.getpgid(test_node_logger.pid), signal.SIGINT)
                test_node_logger.kill()
            except:
                rospy.loginfo('Could not kill logger.')


class StopNodes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['perception_process', 'manipulation_process', 'classifier_process',
                                         'perception_logger_process', 'manipulation_logger_process',
                                         'classifier_logger_process'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Stopping Nodes')
        exterminate(userdata.perception_process.pid, signal.SIGKILL)
        exterminate(userdata.manipulation_process.pid, signal.SIGKILL)
        exterminate(userdata.classifier_process.pid, signal.SIGKILL)
        if userdata.perception_logger_process != 0:
            os.kill(userdata.perception_logger_process.pid, signal.SIGTERM)
        if userdata.manipulation_logger_process != 0:
            os.kill(userdata.manipulation_logger_process.pid, signal.SIGTERM)
        if userdata.classifier_logger_process != 0:
            os.kill(userdata.classifier_logger_process.pid, signal.SIGTERM)
        rospy.signal_shutdown('Finished plan. Shutting down Node.')
        time.sleep(3)
        return 'success'


def exit_handler():
    global manipulation_process
    if manipulation_process != 0:
        print 'Killing manipulation'
        exterminate(manipulation_process.pid, signal.SIGKILL)
    global perception_process
    if perception_process != 0:
        print 'Killing perception'
        exterminate(perception_process.pid, signal.SIGKILL)
    global classifier_process
    if classifier_process != 0:
        print 'Killing classifier'
        exterminate(classifier_process.pid, signal.SIGKILL)
    global manipulation_logger_process
    if manipulation_logger_process != 0:
        print('Killing manipulation logger with pid ' + str(manipulation_logger_process.pid) + '.')
        manipulation_logger_process.terminate()
    global perception_logger_process
    if perception_logger_process != 0:
        print('Killing perception logger with pid ' + str(perception_logger_process.pid) + '.')
        perception_logger_process.terminate()
    global classifier_logger_process
    if classifier_logger_process != 0:
        print('Killing classifier logger with pid' + str(classifier_logger_process.pid) + '.')
        classifier_logger_process.terminate()


atexit.register(exit_handler)