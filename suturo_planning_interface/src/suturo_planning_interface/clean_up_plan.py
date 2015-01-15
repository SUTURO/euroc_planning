import rospy
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_planning_plans.statechooseobject import CleanUpPlan, ChooseObject

__author__ = 'hansa'


class CleanUpService(object):

    SERVICE_NAME = "suturo/state/clean_up_plan"

    def __init__(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)
        self.__clean_up = CleanUpPlan()

    def __handle_request(self, request):
        taskdata = request.taskdata
        result = self.__clean_up.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)


class ChooseObjectService(object):
    SERVICE_NAME = "suturo/state/choose_object"

    def __init__(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)
        self.__choose = ChooseObject()

    def __handle_request(self, request):
        taskdata = request.taskdata
        result = self.__choose.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)