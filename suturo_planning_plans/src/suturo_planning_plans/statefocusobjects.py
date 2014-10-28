import smach
import rospy
import time
from suturo_planning_plans import utils
from suturo_planning_manipulation import calc_grasp_position


class FocusObjects(smach.State):

    _objects_to_focus = None
    _fitted_objects = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'focusObject', 'focusHandle'],
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
                position = userdata.fitted_object.mpe_object.primitive_poses[0].position
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
        next_object = self._objects_to_focus.pop()
        userdata.object_to_focus = next_object

        if utils.is_handle(next_object.object.id, userdata.yaml):
            return 'focusHandle'
        else:
            return 'focusObject'


class FocusObject(smach.State):

    _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'retry', 'fail'],
                             input_keys=['yaml', 'object_to_focus', 'enable_movement'],
                             output_keys=['focused_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FocusObject')
        rospy.loginfo('Trying to focus %s' % userdata.object_to_focus.object.id)

        userdata.focused_object = None

        centroid = userdata.object_to_focus.c_centroid
        centroid.z -= 0.03

        poses = calc_grasp_position.make_scan_pose(centroid, *utils.focus_poses[self._ctr])
        if userdata.enable_movement:
            move_method = utils.manipulation.move_arm_and_base_to
        else:
            utils.manipulation.set_planning_time_arm(2)
            move_method = utils.manipulation.move_to

        for pose in poses:
            if move_method(pose):
                userdata.focused_object = userdata.object_to_focus

                rospy.logdebug('Wait for clock')
                time.sleep(1)

                rospy.logdebug('Wait for tf again.')
                rospy.sleep(4)

                self._ctr = 0
                return 'success'

        utils.manipulation.set_planning_time_arm(5)

        if self._ctr < 6:
            self._ctr += 1
            return 'retry'
        else:
            self._ctr = 0
            return 'fail'


class RefocusHandle(smach.State):

    # _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['focusHandle', 'noHandle'],
                             input_keys=['yaml', 'classified_objects', 'focused_object'],
                             output_keys=['object_to_focus'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FocusHandle')

        filter_handle = [obj for obj in userdata.classified_objects
                         if obj.object.id == userdata.focused_object.object.id]

        if filter_handle:
            userdata.object_to_focus = filter_handle[0]
            return 'focusHandle'
        else:
            return 'noHandle'
