import rospy
from suturo_planning_interface import utils
from suturo_planning_search.map import Map
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceRequest, TaskDataServiceResponse
from suturo_interface_msgs.srv import AddPointCloud, AddPointCloudRequest, AddPointCloudResponse
from suturo_interface_msgs.srv import GetBaseOrigin, GetBaseOriginRequest, GetBaseOriginResponse
from suturo_interface_msgs.srv import EurocObjectToOdomCombined, EurocObjectToOdomCombinedRequest, EurocObjectToOdomCombinedResponse
from suturo_interface_msgs.srv import MarkRegionAsObjectUnderPoint, MarkRegionAsObjectUnderPointRequest, MarkRegionAsObjectUnderPointResponse
from suturo_interface_msgs.srv import CurrentMapToCollisionObject, CurrentMapToCollisionObjectRequest, CurrentMapToCollisionObjectResponse
from suturo_interface_msgs.srv import GetPercentCleared, GetPercentClearedResponse

class MapScanner(object):

    NAME_SERVICE = 'suturo/state/scan_map'
    RETURN_VAL_MAP_SCANNED = 'mapScanned'
    NAME_SERVICE_POINT_CLOUD = '/suturo/add_point_cloud'
    NAME_SERVICE_GET_BASE_ORIGIN = "/suturo/get_base_origin"
    NAME_SERVICE_EUROC_OBJECT_TO_ODOM_COMBINED = "/suturo/euroc_object_to_odom_combined"
    NAME_SERVICE_CURRENT_MAP_TO_COLLISION_OBJECT = "/suturo/current_map_to_collision_object"
    NAME_SERVICE_MARK_REGION_AS_OBJECT_UNDER_POINT = "suturo/mark_region_as_object_under_point"
    NAME_SERVICE_GET_PERCENT_CLEARED = "suturo/map/get_percent_cleared"

    def __init__(self):
        self.create_service()

    def create_service(self):
        rospy.Service(self.NAME_SERVICE, TaskDataService, self._handle_scan_map)
        rospy.Service(self.NAME_SERVICE_POINT_CLOUD, AddPointCloud, self._handle_add_point_cloud)
        rospy.Service(self.NAME_SERVICE_GET_BASE_ORIGIN, GetBaseOrigin, self._handle_get_base_origin)
        rospy.Service(self.NAME_SERVICE_EUROC_OBJECT_TO_ODOM_COMBINED, EurocObjectToOdomCombined,
                      self._handle_euroc_to_combined)
        rospy.Service(self.NAME_SERVICE_CURRENT_MAP_TO_COLLISION_OBJECT, CurrentMapToCollisionObject,
                      self._handle_current_map_to_collision_object)
        rospy.Service(self.NAME_SERVICE_MARK_REGION_AS_OBJECT_UNDER_POINT, MarkRegionAsObjectUnderPoint,
                      self._handle_mark_region_as_object_unter_point)
        rospy.Service(self.NAME_SERVICE_GET_PERCENT_CLEARED, GetPercentCleared,
                      self.__handle_get_percent_cleared)

    def _handle_mark_region_as_object_unter_point(self, req):
        resp = MarkRegionAsObjectUnderPointResponse()
        resp.result = utils.map.mark_region_as_object_under_point(req.x, req.y)
        return resp

    def _handle_current_map_to_collision_object(self, req):
        resp = CurrentMapToCollisionObjectResponse()
        resp.object = utils.map.to_collision_object()
        return resp

    def _handle_euroc_to_combined(self, req):
        print(req)
        print(req.toConvert)
        resp = EurocObjectToOdomCombinedResponse()
        utils.euroc_object_to_odom_combined(req.toConvert)
        resp.converted = req.toConvert
        return resp

    def _handle_get_base_origin(self, req):
        arm_base = utils.manipulation.get_base_origin()
        resp = GetBaseOriginResponse()
        resp.base_origin = arm_base.point
        return resp

    def _handle_add_point_cloud(self, req):
        if utils.map is None:
            utils.map = Map(2)
        if req.arm_origin is not None:
            print(req.arm_origin)
            utils.map.add_point_cloud(req.arm_origin, scene_cam=req.scenecam)
        else:
            utils.map.add_point_cloud(scene_cam=req.scenecam)
        return AddPointCloudResponse()

    def _handle_scan_map(self, req):
        taskdata = req.taskdata
        rospy.loginfo('Scanning map')
        utils.map = Map(2)
        self._scan_map()
        return TaskDataServiceResponse(taskdata = taskdata, result = self.RETURN_VAL_MAP_SCANNED)

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

    def __handle_get_percent_cleared(self, request):
        percent = utils.map.get_percent_cleared()
        return GetPercentClearedResponse(percent=percent)