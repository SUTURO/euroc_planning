import rospy
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse

class SearchObjects():

    RETURN_VAL_MISSING_OBJECTS = 'missingObjects'
    RETURN_VAL_ALL_OBJECTS_FOUND = 'noObjectsLeft'
    NAME_SERVICE = 'suturo/state/search_objects'

    def __init__(self):
        self._found_objects = []
        self._missing_objects = None
        rospy.Service(self.NAME_SERVICE, TaskDataService, self.handle_search_objects)

    def handle_search_objects(self, req):
        """
        Servicehandler for the service NAME_SERVICE
        :param taskdata: The given Servicerequest - Parameter
        :return: The required response object
        """
        rospy.loginfo("Searching objects")
        taskdata = req.taskdata
        self._check_missing_objects(taskdata.yaml.objects)
        self._check_found_objects(taskdata.fitted_objects, taskdata.objects_found)
        self._print_found_objects()
        state_transition = self._check_all_objects_found()
        return TaskDataServiceResponse(taskdata = taskdata, result = state_transition)

    def _check_missing_objects(self, yaml_objects):
        "Searches for missing objects and fills self._missing_objects. This is done once!"
        if self._missing_objects is None:
            self._missing_objects = []
            for obj in yaml_objects:
                print("adding missing object"+obj.name)
                self._missing_objects.append(obj.name)

    def _check_found_objects(self, fitted_objects, objects_found):
        """
        Checks if the objects in fitted_objects are new. If they are, continues to check the new objects with the missing ones. If
        a missing object is found, deletes the object from the list missing_objects und appends it to objects_found
        :param fitted_objects:
        :param objects_found:
        """
        for obj in fitted_objects:
            # Check if the object was already found
            if [x for x in self._found_objects if x.mpe_object.id == obj.mpe_object.id]:
                rospy.loginfo('Object %s was already found.' % obj.mpe_object.id)
            else:
                self._found_objects.append(obj)
                if obj.mpe_object.id in self._missing_objects:
                    self._missing_objects.remove(obj.mpe_object.id)
        objects_found = self._found_objects

    def _print_found_objects(self):
        "Prints all found objects"
        rospy.loginfo('##########################################')
        rospy.loginfo('Objects found:')
        for obj in self._found_objects:
            rospy.loginfo(obj.mpe_object.id)
        rospy.loginfo('##########################################')

    def _check_all_objects_found(self):
        """
        Checks if all objects are found.

        :return: Returns the string RETURN_VAL_ALL_OBJECTS_FOUND if all objects are found. Otherwise returns RETURN_VAL_MISSING_OBJECTS
        """
        if not self._missing_objects:
            return self.RETURN_VAL_ALL_OBJECTS_FOUND
        return self.RETURN_VAL_MISSING_OBJECTS