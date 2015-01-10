import rospy
from suturo_planning_interface import utils
from suturo_planning_search.map import Map
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse
from suturo_interface_msgs.srv import AddPointCloud, AddPointCloudRequest, AddPointCloudResponse

class MapScanner(object):

    NAME_SERVICE = 'suturo/state/scan_map'
    RETURN_VAL_MAP_SCANNED = 'mapScanned'
    NAME_SERVICE_POINT_CLOUD = '/suturo/add_point_cloud'

    def __init__(self):
        self.create_service()

    def create_service(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_scan_map)
        rospy.Service(self.NAME_SERVICE_POINT_CLOUD, AddPointCloud, self._handle_add_point_cloud)

    def _scan_map_part(self, param1, param2):
        utils.manipulation.pan_tilt(param1, param2)
        rospy.sleep(utils.waiting_time_before_scan)
        utils.map.add_point_cloud(scene_cam=True)

    def _handle_add_point_cloud(self, req):
        utils.map = Map(2)
        utils.map.add_point_cloud(scene_cam=req.scenecam)
        return AddPointCloudResponse()

    def _scan_map(self):
        self._scan_map_part(0.2, 0.5)
        self._scan_map_part(0.2825, 0.775)
        self._scan_map_part(0, 1.1)
        self._scan_map_part(-0.2825, 0.775)
        self._scan_map_part(-0.2, 0.5)


    def _handle_scan_map(self, req):
        taskdata = req.taskdata
        rospy.loginfo('Scanning map')
        utils.map = Map(2)
        self._scan_map()
        return TaskDataServiceResponse(taskdata = taskdata, result = self.RETURN_VAL_MAP_SCANNED)
