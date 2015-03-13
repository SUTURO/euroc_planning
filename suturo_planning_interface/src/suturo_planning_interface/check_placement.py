__author__ = 'tobi'
import rospy
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse

class CheckPlacement(object):
    SERVICE_NAME = "suturo/state/check_placement"

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)

    def __handle_request(self, request):
        taskdata = request.taskdata
        result = self.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)

    def execute(self, taskdata):
        taskdata.placed_object = taskdata.object_to_move.mpe_object.id
        return 'onTarget'