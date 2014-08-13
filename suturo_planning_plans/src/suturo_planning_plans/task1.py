import rospy
import smach
import time
from suturo_planning_perception import perception
from suturo_planning_manipulation.manipulation import Manipulation

manipulation = 0


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

    _scans = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['object_to_perceive'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        global manipulation
        if manipulation == 0:
            manipulation = Manipulation()

        # take initial scan pose
        if self._scans == 0:
            self._scans = 1
            manipulation.move_to('scan_pose1')

            # look for objects
            recognized_objects = 0 #perception.recognize_objects_of_interest()
            if 0 != 0:  # check if an object was recognized
                userdata.object_to_perceive = recognized_objects[1]
                return 'objectFound'

        # search for objects
        num_of_scans = 12
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans
        for x in range(self._scans, num_of_scans):
            self._scans = x
            rad = x * rad_per_step - 2.945
            print 'Turning arm ' + str(rad)
            manipulation.turn_arm(1.0, rad)
            time.sleep(2)

            #look for object
            recognized_objects = 0 #perception.recognize_objects_of_interest()
            if 0 != 0:  # check if an object was recognized
                userdata.object_to_perceive = recognized_objects[1]
                return 'objectFound'

        # some code to look into the dead angle
        time.sleep(3)
        return 'noObjectsLeft'


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive'],
                             output_keys=['object_to_grasp'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')
        # awesome code from andz to get the pose to perceive the object

        perceived_object = perception.get_gripper_perception()
        # check if it was an object
        userdata.object_to_grasp = perceived_object
        time.sleep(3)
        return 'validObject'


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['object_to_grasp'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')
        global manipulation
        manipulation.grasp(userdata.object_to_grasp)
        time.sleep(3)
        return 'success'


class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
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
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state ')
        global manipulation
        manipulation.move_to(0)  # again andz's super code

        # something with the gripper cam
        time.sleep(3)
        return 'onTarget'