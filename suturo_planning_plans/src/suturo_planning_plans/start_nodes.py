import rospy
import smach
import time
import subprocess
import os
import suturo_planning_task_selector
import suturo_planning_manipulation


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['manipulation', 'manipulation_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartManipulation')
        userdata.manipulation_process = subprocess.Popen('roslaunch suturo_manipulation_moveit manipulation.launch',
                                                         stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        time.sleep(3)
        userdata.manipulation = suturo_planning_manipulation.Manipulation()
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['perception_process'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        userdata.perception_process = subprocess.Popen('roslaunch euroc_launch perception_task1.launch',
                                                       stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
        time.sleep(3)
        return 'success'


class StartSimulation(smach.State):
    task_name = ''

    def __init__(self, task_name):
        self.task_name = task_name
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['yaml', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        userdata.yaml = suturo_planning_task_selector.start_task(self.task_name)
        userdata.objects_found = []
        time.sleep(3)
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