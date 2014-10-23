import sys
import rospy
import smach
import time
import subprocess
import os
import signal
import atexit
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
                             input_keys=['initialization_time'],
                             output_keys=['manipulation_process', 'manipulation_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing TestNode init.')
        subprocess.Popen('rosrun euroc_launch TestNode --init', shell=True)
        rospy.loginfo('Executing state StartManipulation')
        global manipulation_process
        manipulation_process = subprocess.Popen('roslaunch euroc_launch manipulation.launch',
                                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                                shell=True, preexec_fn=os.setsid)
        userdata.manipulation_process = manipulation_process
        global manipulation_logger_process
        manipulation_logger_process = subprocess.Popen('./logger.py "' + utils.log_dir + '/' +
                                                       userdata.initialization_time +
                                                       ' Manipulation.log"', stdin=manipulation_process.stdout,
                                                       shell=True, preexec_fn=os.setsid)
        userdata.manipulation_logger_process = manipulation_logger_process
        time.sleep(5)
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time'],
                             output_keys=['perception_process', 'perception_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        global perception_process
        perception_process = subprocess.Popen('roslaunch euroc_launch perception_task1.launch',
                                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                              shell=True, preexec_fn=os.setsid)
        userdata.perception_process = perception_process
        global perception_logger_process
        perception_logger_process = subprocess.Popen('./logger.py "' + utils.log_dir + '/' +
                                                     userdata.initialization_time +
                                                     ' Perception.log"', stdin=perception_process.stdout,
                                                     shell=True, preexec_fn=os.setsid)
        userdata.perception_logger_process = perception_logger_process
        time.sleep(8)
        return 'success'


class StartPerceptionTask6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time'],
                             output_keys=['perception_process', 'perception_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerceptionTask6')
        global perception_process
        perception_process = subprocess.Popen('roslaunch euroc_launch perception_task6.launch',
                                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                              shell=True, preexec_fn=os.setsid)
        userdata.perception_process = perception_process
        global perception_logger_process
        perception_logger_process = subprocess.Popen('./logger.py "' + utils.log_dir + '/' +
                                                     userdata.initialization_time +
                                                     ' Perception.log"', stdin=perception_process.stdout,
                                                     shell=True, preexec_fn=os.setsid)
        userdata.perception_logger_process = perception_logger_process
        time.sleep(8)
        return 'success'


class StartClassifier(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['initialization_time'],
                             output_keys=['classifier_process', 'classifier_logger_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartClassifier')
        global classifier_process
        classifier_process = subprocess.Popen('rosrun suturo_perception_classifier classifier.py',
                                              stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                              shell=True, preexec_fn=os.setsid)
        userdata.classifier_process = classifier_process
        global classifier_logger_process
        classifier_logger_process = subprocess.Popen('./logger.py "' + utils.log_dir + '/' +
                                                     userdata.initialization_time +
                                                     ' Classifier.log"', stdin=classifier_process.stdout,
                                                     shell=True, preexec_fn=os.setsid)
        userdata.classifier_logger_process = classifier_logger_process
        rospy.sleep(8)
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
        userdata.objects_found = []
        self.clock = rospy.Subscriber('clock', Clock, self.wait_for_clock())
        rospy.loginfo('Waiting for clock.')
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
                             input_keys=['yaml', 'initialization_time'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state StopSimulation')
        rospy.loginfo('Executing TestNode check.')
        test_node = subprocess.Popen('rosrun euroc_launch TestNode --check', shell=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        test_node_logger = subprocess.Popen('./logger.py "' + utils.log_dir + '/' + userdata.initialization_time +
                                            ' TestNode check.log"', shell=True, stdin=test_node.stdout)
        test_node.wait()
        rospy.loginfo('Killing TestNode logger.')
        os.kill(test_node_logger.pid, signal.SIGTERM)
        if self.savelog:
            save_task()
        stop_task()
        time.sleep(3)
        return 'success'


class StopNodes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['perception_process', 'manipulation_process', 'classifier_process',
                                         'perception_logger_process', 'manipulation_loger_process',
                                         'classifier_logger_process'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Stopping Nodes')
        os.killpg(userdata.perception_process.pid, signal.SIGTERM)
        os.killpg(userdata.manipulation_process.pid, signal.SIGTERM)
        os.killpg(userdata.classifier_process.pid, signal.SIGTERM)
        os.killpg(userdata.perception_logger_process.pid, signal.SIGTERM)
        os.killpg(userdata.manipulation_logger_process.pid, signal.SIGTERM)
        os.killpg(userdata.classifier_loger_process.pid, signal.SIGTERM)
        rospy.signal_shutdown('Finished plan. Shutting down Node.')
        time.sleep(3)
        return 'success'


def exit_handler():
    global manipulation_process
    if manipulation_process != 0:
        print 'Killing manipulation'
        os.killpg(manipulation_process.pid, signal.SIGTERM)
    global perception_process
    if perception_process != 0:
        print 'Killing perception'
        os.killpg(perception_process.pid, signal.SIGTERM)
    global classifier_process
    if classifier_process != 0:
        print 'Killing classifier'
        os.killpg(classifier_process.pid, signal.SIGTERM)
    global manipulation_logger_process
    if manipulation_logger_process != 0:
        print 'Killing manipulation logger'
        os.killpg(manipulation_logger_process.pid, signal.SIGTERM)
    global perception_logger_process
    if perception_logger_process != 0:
        print 'Killing perception logger'
        os.killpg(perception_logger_process.pid, signal.SIGTERM)
    global classifier_logger_process
    if classifier_logger_process != 0:
        print 'Killing classifier logger'
        os.killpg(classifier_logger_process.pid, signal.SIGTERM)


atexit.register(exit_handler)