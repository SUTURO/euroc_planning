from copy import deepcopy
import rospy
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_planning_interface import utils

__author__ = 'tobi'

class ChooseObject(object):
    SERVICE_NAME = "suturo/state/choose_object"

    _ctr = 0
    failed_objects = []
    retry = False

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)

    def __handle_request(self, request):
        taskdata = request.taskdata
        #taskdata.clean_up_plan = map(lambda action: (action.object, action), taskdata.clean_up_plan)
        result = self.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')
        rospy.loginfo("number failed objects: " + str(len(self.failed_objects)))

        #if not self.retry and userdata.failed_object is not None and userdata.failed_object not in self.failed_objects:

            #self.failed_objects.append(userdata.failed_object)

        self.retry = False

        if len(userdata.clean_up_plan) > self._ctr:
            action = userdata.clean_up_plan[self._ctr]
            userdata.object_to_move = action.object
            self._ctr += 1
        else:
            self._ctr = 0
            if self.failed_objects:
                rospy.loginfo("retry to place failed objects:")
                rospy.loginfo("################################")
                rospy.loginfo(str(self.failed_objects))
                rospy.loginfo("################################")
                userdata.objects_found = deepcopy(self.failed_objects)

                self.failed_objects = []
                if userdata.enable_movement:
                    utils.manipulation.move_arm_and_base_to([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0])
                else:
                    utils.manipulation.move_arm_and_base_to([0.0,0.0,0.0,0.0,0.0,0.0,0.0])

                rospy.logwarn("not all objects where grapsed/placed, retry. if it crashes now it doesnt matter")
                self.retry = True
                return 'retry'
            return 'noObjectsLeft'

        rospy.loginfo('Placing object on location %s' % action.target_position)
        userdata.place_position = action.target_position

        return 'objectChosen'
