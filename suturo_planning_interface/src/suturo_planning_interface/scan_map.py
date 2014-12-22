import rospy
import suturo_planning_plans.utils as utils
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.mathemagie import set_vector_length, add_point
from suturo_planning_search.map import Map
from suturo_planning_manipulation.manipulation import Manipulation
from math import *
from suturo_msgs.msg import Task
from std_srvs.srv import Empty

from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse

class MapScanner(object):

    NAME_SERVICE = 'suturo/state/scan_map'
    RETURN_VAL_MAP_SCANNED = 'mapScanned'

    def __init__(self):
        self.create_service()

    def create_service(self):
        s = rospy.Service(self.NAME_SERVICE, TaskDataService, self.scan_map)

    def _scan_map_part(self, param1, param2):
        utils.manipulation.pan_tilt(param1, param2)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)

    def _scan_map(self):
        self._scan_map_part(0.2, 0.5)
        self._scan_map_part(0.2825, 0.775)
        self._scan_map_part(0, 1.1)
        self._scan_map_part(-0.2825, 0.775)
        self._scan_map_part(-0.2, 0.5)


    def _handle_scan_map(self, taskdata):
        rospy.loginfo('Scanning map')
        self._scan_map()
        utils.map = Map(2)
        return TaskDataServiceResponse(taskdata = taskdata, result = self.RETURN_VAL_MAP_SCANNED)
