import rospy
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_perception_msgs.msg import EurocObject
from suturo_planning_perception import perception
from suturo_planning_interface.utils import get_valid_objects, classify_object, euroc_object_to_odom_combined


class ClassifyObjects(object):
    def __init__(self, service_name="suturo/state/scan_obstacles"):
        rospy.Service(service_name, TaskDataService, self.__handle_call)

    def __handle_call(self, msg):
        response = TaskDataServiceResponse()
        response.result = self.__classify(msg.taskdata)
        response.taskdata = msg.taskdata
        return response

    def __classify(self, userdata):
        rospy.loginfo('Executing state ClassifyObject')
        userdata.sec_try = False

        rospy.loginfo('Using simple perception.')
        get_gripper = perception.get_gripper_perception()
        if get_gripper is None:
            rospy.logwarn("get_gripper is None!!!?!?")
            get_gripper = []

        perceived_objects = get_valid_objects(get_gripper)
        rospy.logdebug('Found ' + str(len(get_gripper)) + ' objects, ' + str(len(perceived_objects)) + ' are valid.')
        if not perceived_objects:
            if not userdata.sec_try_done:
                userdata.sec_try = True
            return 'noObject'

        collision_objects = []
        matched_objects = []
        found_object_names = []
        for obj in userdata.objects_found:
            found_object_names.append(obj.mpe_object.id)

        for obj in perceived_objects:
            # classify object
            matched_obj = classify_object(obj)
            rospy.logdebug('Matched: \n' + str(obj) + '\n----------------------------------\nwith: ' + str(matched_obj))

            if matched_obj.c_type == EurocObject.OBSTACLE:
                if matched_obj.c_cuboid_success:
                    collision_objects.append(matched_obj.object)
                continue

            if matched_obj.c_type == EurocObject.UNKNOWN or matched_obj.c_type == EurocObject.TABLE:
                if matched_obj.c_cuboid_success:
                    collision_objects.append(matched_obj.object)
                # TODO something better than ignoring the problem
                rospy.logwarn('Unknown object perceived or table. Ignoring it for now.')
                continue

            if matched_obj.c_type == EurocObject.OBJECT:
                # check if the object was already placed
                rospy.loginfo('Matched object ' + str(matched_obj.object.id) + '.')
                if matched_obj.object.id in found_object_names:
                    rospy.loginfo('Object %s was already seen.' % matched_obj.object.id)
                else:
                    euroc_object_to_odom_combined(matched_obj)
                    matched_objects.append(matched_obj)

        # publish_collision_objects(collision_objects)
        userdata.classified_objects = matched_objects

        # check if it was an object
        if matched_objects:
            return 'objectsClassified'
        else:
            if not userdata.sec_try_done:
                userdata.sec_try = True
            return 'noObject'