import rospy
import smach
import time
import subprocess
import os
import atexit
import suturo_planning_task_selector
from suturo_planning_manipulation.manipulation import Manipulation


perception_process = 0
manipulation_process = 0


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['manipulation', 'manipulation_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartManipulation')
        global manipulation_process
        manipulation_process = subprocess.Popen('roslaunch euroc_launch manipulation.launch',
                                                stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        userdata.manipulation_process = manipulation_process
        time.sleep(3)
        userdata.manipulation = Manipulation()
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
        userdata.perception_process.kill()
        userdata.manipulation_process.kill()
        time.sleep(3)
        return 'success'


def exit_handler():
    time.sleep(2)
    print 'Killing perception and manipulation'
    global manipulation_process
    if manipulation_process != 0:
        manipulation_process.kill()
    global perception_process
    if perception_process != 0:
        perception_process.kill()


atexit.register(exit_handler)