import rospy
from suturo_interface_msgs.srv import SearchObjects

class SearchObjects():

    def __init__(self):
        self._found_objects = []
        self._missing_objects = None
        search_objects_server()

    def search_objects_server(self):
        rospy.init_node('seach_objects_server')
        service = rospy.Service('search_objects', SearchObjects, self.handle_search_objects) 

    def handle_search_objects(self):
        rospy.loginfo("Searching objects")
        self._check_missing_objects()
        self._check_found_objects()
        if not self._missing_objects:
            return SearchObjects("Done")

    def _check_missing_objects(self):
        "Searches for missing objects and fills self._missing_objects. This is done only once!"
        if self._missing_objects is None:
            self._missing_objects = []
            for obj in userdata.yaml.objects:
                print("adding missing object"+obj.name)
                self._missing_objects.append(obj.name)

    def _check_found_objects(self):
        "Checks the found objects."
        for obj in userdata.fitted_objects:
            # Check if the object was already found
            if [x for x in self._found_objects if x.mpe_object.id == obj.mpe_object.id]:
                rospy.loginfo('Object %s was already found.' % obj.mpe_object.id)
            else:
                self._found_objects.append(obj)
                if obj.mpe_object.id in self._missing_objects:
                    self._missing_objects.remove(obj.mpe_object.id)
        userdata.objects_found = self._found_objects

    if __name__ == "__main__":
        obj = SearchObjects()
