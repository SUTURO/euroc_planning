import smach
import rospy
import time
import utils
from suturo_planning_manipulation import calc_grasp_position, mathemagie
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse


class FocusObjects():

    NAME_SERVICE = 'suturo/state/focus_objects'
    RETURN_VAL_NO_MORE_OBJECTS_TO_FOCUS = 'success'
    RETURN_VAL_STILL_MORE_OBJECTS_TO_FOCUS = ""
    RETURN_VAl_FOCUS_HANDLE = "focusHandle"
    RETURN_VAL_FOCUS_OBJECT = "focusObject"

    _objects_to_focus = None
    _fitted_objects = []

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_focus_object)

    def _handle_focus_object(self, taskdata):
        self._read_objects_to_focus(taskdata.objects_to_focus, taskdata.focused_point, taskdata.fitted_object)
        transition = self._check_objects_to_focus(taskdata.fitted_objects)
        if transition == self.RETURN_VAL_STILL_MORE_OBJECTS_TO_FOCUS:
            transition = self._set_next_object_to_focus(taskdata.object_to_focus, taskdata.yaml)
        return TaskDataServiceResponse(taskdata = taskdata, result = transition)

    def _read_objects_to_focus(self, objects_to_focus, focused_point, fitted_object):
        "This is done once!"
        # Read the objects_to_focus if this is the first iteration
        if self._objects_to_focus is None:
            self._objects_to_focus = objects_to_focus
            if not focused_point is None:
                object_to_focus = None
                min_dist = 100
                for obj in self._objects_to_focus:
                    dist = mathemagie.euclidean_distance(obj.c_centroid, focused_point)
                    if dist < min_dist:
                        min_dist = dist
                        object_to_focus = obj
                self._objects_to_focus = [object_to_focus]
        elif not focused_point is None:
            self._fitted_objects.append(fitted_object)
            if not utils.map is None:
                position = fitted_object.mpe_object.primitive_poses[0].position
                utils.map.mark_region_as_object_under_point(position.x, position.y)
                rospy.logdebug(str(utils.map))
                co = utils.map.to_collision_object()
                utils.manipulation.get_planning_scene().add_object(co)

    def _check_objects_to_focus(self, fitted_objects):
        # If there are no more objects to focus
        if not self._objects_to_focus:
            self._objects_to_focus = None
            fitted_objects = self._fitted_objects
            self._fitted_objects = []
            return self.RETURN_VAL_NO_MORE_OBJECTS_TO_FOCUS
        else:
            return self.RETURN_VAL_STILL_MORE_OBJECTS_TO_FOCUS

    def _set_next_object_to_focus(self, object_to_focus, yaml):
        "Set the next object to focus"
        next_object = self._objects_to_focus.pop()
        object_to_focus = next_object
        if utils.is_handle(next_object.object.id, yaml):
            return self.RETURN_VAl_FOCUS_HANDLE
        else:
            return self.RETURN_VAL_FOCUS_OBJECT


class FocusObject():

    _ctr = 0

    NAME_SERVICE = 'suturo/state/focus_object'
    RETURN_VAL_SUCSS = 'success'
    RETURN_VAL_FAIL = 'fail'
    RETURN_VAL_RETRY = "retry"

    def __init__(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_focus)

    def _handle_focus(self, taskdata):
        rospy.loginfo('Executing state FocusObject')
        rospy.loginfo('Trying to focus %s' % taskdata.object_to_focus.object.id)
        taskdata.focused_object = None
        centroid = taskdata.object_to_focus.c_centroid
        centroid.z -= 0.03
        poses = calc_grasp_position.make_scan_pose(centroid, *utils.focus_poses[self._ctr])
        utils.manipulation.set_planning_time_arm(2)
        move_method = utils.manipulation.move_to

        transition = ""
        for pose in poses:
            if move_method(pose):
                taskdata.focused_object = taskdata.object_to_focus
                rospy.logdebug('Wait for clock')
                time.sleep(1)
                rospy.logdebug('Wait for tf again.')
                rospy.sleep(4)
                self._ctr = 0
                transition = self.RETURN_VAL_SUCSS
        if transition == "":
            utils.manipulation.set_planning_time_arm(5)
            if self._ctr < 6:
                self._ctr += 1
                transition = self.RETURN_VAL_RETRY
            else:
                self._ctr = 0
                transition = self.RETURN_VAL_FAIL

        return TaskDataServiceResponse(taskdata = taskdata, result = transition)


class RefocusHandle():

    # _ctr = 0
    NAME_SERVICE = 'suturo/state/refocus_handle'

    def __init__(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_refocus)

    def _handle_refocus(self, taskdata):
        rospy.loginfo('Executing state FocusHandle')
        filter_handle = [obj for obj in taskdata.classified_objects if obj.object.id == taskdata.focused_object.object.id]
        if filter_handle:
            taskdata.object_to_focus = filter_handle[0]
            transition = 'focusHandle'
        else:
            transition = 'noHandle'

        return TaskDataServiceResponse(taskdata = taskdata, result = transition)
