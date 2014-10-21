import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation import manipulation_service

import utils
from suturo_planning_manipulation.manipulation import Manipulation


class SearchObject(smach.State):
    _found_objects = []
    _missing_objects = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['searchObject', 'noObjectsLeft', 'simStopped'],
                             input_keys=['yaml', 'perceived_objects', 'task', 'fitted_objects', 'enable_movement'],
                             output_keys=['objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        if self._missing_objects is None:
            self._missing_objects = []
            for obj in userdata.yaml.objects:
                self._missing_objects.append(obj.name)

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        # Add the found objects
        for obj in userdata.fitted_objects:
            # Check if the object was already found
            if [x for x in self._found_objects if x.mpe_object.id == obj.mpe_object.id]:
                rospy.loginfo('Object %s was already found.' % obj.mpe_object.id)
            else:
                self._found_objects.append(obj)
                if obj.mpe_object.id in self._missing_objects:
                    self._missing_objects.remove(obj.mpe_object.id)
        userdata.objects_found = self._found_objects

        # Check if all objects were found
        if not self._missing_objects:
            return 'noObjectsLeft'

        return 'searchObject'