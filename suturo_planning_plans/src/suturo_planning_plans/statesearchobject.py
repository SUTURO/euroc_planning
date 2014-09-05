import smach
import time
import rospy
import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception


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

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            time.sleep(2)

        # Add the found objects
        if userdata.objects_found:
            self._found_objects = userdata.objects_found
            # Stop searching if all objects were found
            if len(self._found_objects) == len(userdata.yaml.objects):
                return 'noObjectsLeft'

            # After placing an object go back into the scan_pose1
            utils.manipulation.move_to('scan_pose1')

        # take initial scan pose
        if self._next_scan == 0:
            print 'Take scan pose 1'
            utils.manipulation.move_to('scan_pose1')

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
                utils.manipulation.turn_arm(1.0, rad)

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