import rospy
import smach
import time
from utils import *
from suturo_planning_perception import perception
from suturo_planning_manipulation.manipulation import Manipulation
from geometry_msgs.msg import *

# Holds the manipulation object
_manipulation = None


class Task1(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
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
                                                'notOnTarget': 'GraspObject',
                                                'nextObject': 'GraspObject'})

        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []


class SearchObject(smach.State):

    _next_scan = 0
    _found_objects = []
    _recognized_objects = []
    _obj_colors = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['object_to_perceive'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        if not self._obj_colors:
            for obj in userdata.yaml.objects:
                self._obj_colors.append(hex_to_color_msg(obj.color))

        # TODO find a way to take last pose
        # First check previously recognized objects
        # if self._recognized_objects:
        #     userdata.object_to_perceive = self._recognized_objects.pop(0)
        #     return 'objectFound'

        global _manipulation
        if _manipulation is None:
            _manipulation = Manipulation()
            time.sleep(2)

        # Add the found objects
        if userdata.objects_found:
            self._found_objects = userdata.objects_found
            _manipulation.move_to('scan_pose1')

        # take initial scan pose
        if self._next_scan == 0:
            print 'Take scan pose 1'
            _manipulation.move_to('scan_pose1')

        # get the colors of the missing objects, assuming for now that every object has its own color
        colors = self._obj_colors
        for obj in self._found_objects:
            try:
                colors.remove(obj.color)
            except ValueError:
                pass

        # search for objects
        num_of_scans = 12
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans

        for x in range(self._next_scan, num_of_scans):
            # skip turning arm on first scan
            if self._next_scan != 0:
                rad = x * rad_per_step - 2.945
                print 'Turning arm ' + str(rad)
                _manipulation.turn_arm(1.0, rad)

            self._next_scan += 1

            # look for objects
            print 'Colors: ' + str(colors)
            self._recognized_objects = perception.recognize_objects_of_interest(colors)
            print 'Found objects: ' + str(self._recognized_objects)
            if self._recognized_objects:  # check if an object was recognized
                userdata.object_to_perceive = self._recognized_objects.pop(0)

                # Might help with tf
                time.sleep(2)

                return 'objectFound'

        return 'noObjectsLeft'


class PerceiveObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['noObject', 'validObject'],
                             input_keys=['object_to_perceive', 'yaml'],
                             output_keys=['object_to_move', 'pending_objects', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PerceiveObject')

        perceived_objects = get_valid_objects(perception.get_gripper_perception())
        if not perceived_objects:
            userdata.objects_found = []
            return 'noObject'

        collision_objects = []
        for obj in perceived_objects:
            # obj.object.id = str(obj.c_centroid.x)
            matched_obj = match_object(obj, userdata.yaml)
            rospy.logdebug('Matched: ' + str(obj) + '\nwith: ' + str(matched_obj))
            collision_objects.append(obj.object)
            # obj.color = userdata.object_to_perceive.color
        publish_collision_objects(collision_objects)
        userdata.objects_found = perceived_objects

        # check if it was an object
        object_candidate = get_object_to_move(perceived_objects)
        if object_candidate is not None:
            userdata.object_to_move = object_candidate
            return 'validObject'
        else:
            return 'noObject'


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['object_to_move'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')

        global _manipulation
        print 'Trying to grasp:\n' + str(userdata.object_to_move.object.id)
        grasp_result = _manipulation.grasp(userdata.object_to_move.object)
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
        global _manipulation

        destination = PointStamped()
        destination.header.frame_id = '/odom_combined'
        destination.point = userdata.yaml.target_zones[0].target_position
        _manipulation.place(destination)

        return 'success'


class CheckPlacement(smach.State):

    _placed_objects = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['onTarget', 'notOnTarget', 'nextObject'],
                             input_keys=['yaml', 'object_to_move', 'pending_objects'],
                             output_keys=['placed_objects'])

    def execute(self, userdata):

        self._placed_objects.append(userdata.object_to_move)
        userdata.placed_objects = self._placed_objects

        if userdata.pending_objects:
            return 'nextObject'

        return 'onTarget'