import smach
import rospy
from math import pi
import time
from suturo_planning_plans import utils
from suturo_planning_manipulation import calc_grasp_position


class FocusObjects(smach.State):

    _objects_to_focus = None
    _fitted_objects = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'nextObject'],
                             input_keys=['yaml', 'objects_to_focus', 'fitted_object'],
                             output_keys=['object_to_focus', 'fitted_objects'])

    def execute(self, userdata):

        # Read the objects_to_focus if this is the first iteration
        if self._objects_to_focus is None:
            self._objects_to_focus = userdata.objects_to_focus
        # Remember the fitted objects
        elif not userdata.fitted_object is None:
            self._fitted_objects.append(userdata.fitted_object)

            if not utils.map is None:
                position = userdata.fitted_object.mpe_object.primitive_poses[0]
                utils.map.mark_region_as_object_under_point(position.x, position.y)
                co = utils.map.to_collision_object()
                utils.manipulation.get_planning_scene().add_object(co)

        # If there are no more objects to focus
        if not self._objects_to_focus:
            self._objects_to_focus = None
            userdata.fitted_objects = self._fitted_objects
            self._fitted_objects = []
            return 'success'

        # Set the next object to focus
        userdata.object_to_focus = self._objects_to_focus.pop()

        return 'nextObject'


class FocusObject(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'object_to_focus', 'enable_movement'],
                             output_keys=['focused_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FocusObject')
        rospy.loginfo('Trying to focus %s' % userdata.object_to_focus.object.id)

        userdata.focused_object = None

        centroid = userdata.object_to_focus.c_centroid
        centroid.z -= 0.05

        poses = calc_grasp_position.make_scan_pose(centroid, 0.6, pi / 4.0)
        for pose in poses:
            if utils.manipulation.move_to(pose):
                userdata.focused_object = userdata.object_to_focus

                rospy.logdebug('Wait for clock')
                time.sleep(1)

                rospy.logdebug('Wait for tf again.')
                rospy.sleep(4)

                return 'success'

        return 'fail'