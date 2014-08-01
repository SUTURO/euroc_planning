import rospy
import smach
import time


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartManipulation')
        time.sleep(3)
        return 'success'


class StartPerception(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartPerception')
        time.sleep(3)
        return 'success'


class StartSimulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        time.sleep(3)
        return 'success'


class StopSimulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartSimulation')
        time.sleep(3)
        return 'success'