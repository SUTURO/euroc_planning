import rospy
import smach
import smach_ros
import time


class Task1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['outcome1', 'outcome2'])
        with self:
            smach.StateMachine.add('StartManipulation', StartManipulation(),
                                   transitions={'outcome6': 'outcome1',
                                                'outcome7': 'outcome2'})


class StartManipulation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome6', 'outcome7'])

    def execute(self, userdata):
        rospy.loginfo('Executing state StartManipulation')
        return 'outcome6'