import rospy
import smach
import time
from utils import *
from suturo_planning_perception import perception
from suturo_planning_manipulation.manipulation import Manipulation

manipulation = None


class Task1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml', 'objects_found'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'PerceiveObject',
                                                'noObjectsLeft': 'success'})
            smach.StateMachine.add('PerceiveObject', PerceiveObject(),
                                   transitions={'validObject': 'GraspObject',
                                                'noObject': 'SearchObject'})
            smach.StateMachine.add('GraspObject', GraspObject(),
                                   transitions={'success': 'PlaceObject',
                                                'fail': 'GraspObject'})
            smach.StateMachine.add('PlaceObject', PlaceObject(),
                                   transitions={'success': 'CheckPlacement',
                                                'fail': 'PerceiveObject'})
            smach.StateMachine.add('CheckPlacement', CheckPlacement(),
                                   transitions={'onTarget': 'SearchObject',
                                                'notOnTarget': 'GraspObject'})


class SearchObject(smach.State):

    _next_scan = 0
    _found_objects = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['object_to_perceive'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        global manipulation
        if manipulation is None:
            manipulation = Manipulation()

        if len(self._found_objects) > 0:
            userdata.object_to_perceive = self._found_objects.pop(0)
            return 'objectFound'

        # take initial scan pose
        if self._next_scan == 0:
            manipulation.move_to('scan_pose1')

        # search for objects
        num_of_scans = 12
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans
        for x in range(self._next_scan, num_of_scans):
            self._next_scan += 1
            rad = x * rad_per_step - 2.945
            print 'Turning arm ' + str(rad)
            manipulation.turn_arm(1.0, rad)
            time.sleep(1)

            # look for objects
            recognized_objects = [] # perception.recognize_objects_of_interest()
            if len(recognized_objects) > 0:  # check if an object was recognized
                userdata.object_to_perceive = recognized_objects.pop(0)
                self._found_objects = recognized_objects
                return 'objectFound'

        # some code to look into the dead angle
        time.sleep(3)
        return 'noObjectsLeft'


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive'],
                             output_keys=['object_to_move'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        # awesome code from andz to get the pose to perceive the object coming soon
        userdata.object_to_perceive.pose.pose.position.z += 12  # assuming z is the height
        global manipulation
        manipulation.move_to(userdata.object_to_perceive.pose)

        perceived_objects = perception.get_gripper_perception()

        # check if it was an object
        userdata.object_to_move = get_object_to_move(perceived_objects)
        time.sleep(3)
        return 'validObject'


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')
        global manipulation
        manipulation.grasp(userdata.object_to_move.object)
        time.sleep(3)
        return 'success'


class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')
        global manipulation

        destination = 0  # get the target zone from the yaml
        manipulation.place(destination)
        time.sleep(3)
        return 'success'


class CheckPlacement(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['onTarget', 'notOnTarget'],
                             input_keys=['yaml', 'object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ')
        global manipulation
        manipulation.move_to(0)  # again andz's super code

        # something with the gripper cam
        time.sleep(3)
        return 'onTarget'