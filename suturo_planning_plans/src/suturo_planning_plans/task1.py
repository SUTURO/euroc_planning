import rospy
import smach
import time
from utils import *
from suturo_planning_perception import perception
from suturo_planning_manipulation.manipulation import Manipulation
from geometry_msgs.msg import *

# Holds the manipulation object
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
            time.sleep(2)

        if len(self._found_objects) > 0:
            userdata.object_to_perceive = self._found_objects.pop(0)
            return 'objectFound'

        # take initial scan pose
        if self._next_scan == 0:
            print 'Take scan pose 1'
            manipulation.move_to('scan_pose1')

        # get the colors of the objects
        colors = []
        for obj in userdata.yaml.objects:
            colors.append(hex_to_color_msg(obj.color))

        # search for objects
        num_of_scans = 12
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans

        for x in range(self._next_scan, num_of_scans):
            # skip turning arm on first scan
            if self._next_scan != 0:
                rad = x * rad_per_step - 2.945
                print 'Turning arm ' + str(rad)
                manipulation.turn_arm(1.0, rad)

            self._next_scan += 1

            # look for objects
            print 'Colors: ' + str(colors)
            recognized_objects = perception.recognize_objects_of_interest(colors)
            print 'Found objects: ' + str(recognized_objects)
            if len(recognized_objects) > 0:  # check if an object was recognized
                userdata.object_to_perceive = recognized_objects.pop(0)
                self._found_objects = recognized_objects
                return 'objectFound'

        return 'noObjectsLeft'


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive'],
                             output_keys=['object_to_move'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        # awesome code from andz to get the pose to perceive the object coming soon
        # userdata.object_to_perceive.pose.pose.position.x += 12  # assuming x is the height
        # global manipulation
        # manipulation.move_to(userdata.object_to_perceive.pose)

        perceived_objects = perception.get_gripper_perception()
        collision_objects = []
        for obj in perceived_objects:
            obj.object.id = str(obj.c_centroid.x)
            collision_objects.append(obj.object)
        publish_collision_objects(collision_objects)

        print 'Perceived objects: ' + str(perceived_objects)
        print 'Selected object: ' + str(get_object_to_move(perceived_objects))

        # check if it was an object
        userdata.object_to_move = get_object_to_move(perceived_objects)
        return 'validObject'


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')
        global manipulation
        print 'Trying to grasp:\n' + str(userdata.object_to_move.object.id)
        grasp_result = manipulation.grasp(userdata.object_to_move.object)
        print 'Grasp result:' + str(grasp_result)

        if grasp_result:
            return 'success'
        else:
            return 'fail'


class PlaceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlaceObject')
        global manipulation

        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        destination.point = userdata.yaml.target_zones[0].target_position
        manipulation.place(destination)

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