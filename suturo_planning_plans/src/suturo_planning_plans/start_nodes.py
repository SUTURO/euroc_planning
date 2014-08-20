import rospy
import smach
import time
import subprocess
import os
import signal
import atexit
import suturo_planning_task_selector


perception_process = 0
manipulation_process = 0


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['manipulation_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartManipulation')
        global manipulation_process
        manipulation_process = subprocess.Popen('roslaunch euroc_launch manipulation.launch',
                                                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        userdata.manipulation_process = manipulation_process
        time.sleep(5)
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['perception_process', 'test'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        global perception_process
        perception_process = subprocess.Popen('roslaunch euroc_launch perception_task1.launch',
                                              stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        userdata.perception_process = perception_process
        time.sleep(8)
        return 'success'


class StartSimulation(smach.State):
    task_name = ''

    def __init__(self, task_name):
        self.task_name = task_name
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['test'],
                             output_keys=['yaml', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        userdata.yaml = suturo_planning_task_selector.start_task(self.task_name)
        userdata.objects_found = []
        time.sleep(8)
        return 'success'


class StopSimulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state StopSimulation')
        time.sleep(3)
        return 'success'


class StopNodes(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['perception_process', 'manipulation_process'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Stopping Nodes')
        os.killpg(userdata.perception_process.pid, signal.SIGTERM)
        os.killpg(userdata.manipulation_process.pid, signal.SIGTERM)
        time.sleep(3)
        return 'success'


def exit_handler():
    time.sleep(2)
    print 'Killing perception and manipulation'
    global manipulation_process
    if manipulation_process != 0:
        os.killpg(manipulation_process.pid, signal.SIGTERM)
    global perception_process
    if perception_process != 0:
        os.killpg(perception_process.pid, signal.SIGTERM)


atexit.register(exit_handler)