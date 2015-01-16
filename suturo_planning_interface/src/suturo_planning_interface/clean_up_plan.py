from copy import deepcopy
from geometry_msgs.msg import PointStamped
import rospy
from std_msgs.msg import Header
from suturo_interface_msgs.srv import TaskDataService, TaskDataServiceResponse
from suturo_msgs.msg import Task, TargetZone
from suturo_planning_interface import mathemagie
from suturo_planning_interface import utils
from suturo_planning_plans.statechooseobject import CleanUpPlan, ChooseObject

__author__ = 'hansa'


class CleanUpService(object):

    SERVICE_NAME = "suturo/state/clean_up_plan"

    def __init__(self):
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, TaskDataService, self.__handle_request)

    def __handle_request(self, request):
        taskdata = request.taskdata
        result = self.execute(taskdata)
        return TaskDataServiceResponse(taskdata=taskdata, result=result)

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanCleanUp')

        header = Header()
        header.frame_id = '/odom_combined'
        plan = []
        target_zones = []
        if (userdata.yaml.task_type == Task.TASK_5):
            utils.map.remove_puzzle_fixture(yaml=userdata.yaml)
            for puzzle_part in userdata.yaml.relative_puzzle_part_target_poses:
                target_position = mathemagie.add_point(userdata.yaml.puzzle_fixture.position, puzzle_part.pose.position)
                fake_target_zone = TargetZone(name = puzzle_part.name + "_target", expected_object = puzzle_part.name, target_position = target_position, max_distance = 0.05)
                # setting a z value here is useless, will be overwritten later...
                #fake_target_zone.target_position.z = fake_target_zone.target_position.z + 0.5 # put it above the fixture to avoid collision
                print("fake_target_zone = " + str(fake_target_zone))
                target_zones.append(fake_target_zone)
        else:
            target_zones = userdata.yaml.target_zones

        tzs_for = {tz.expected_object: tz for tz in target_zones}
        found_objects = userdata.objects_found

        print ("tzs_for = " + str(tzs_for))

        # Get the objects that are in an others objects target zone
        objects_in_tzs = []
        for obj in found_objects:
            tz = utils.in_target_zone(obj, userdata.yaml)
            if not tz is None and tz.expected_object != obj.mpe_object.id:
                objects_in_tzs.append((obj, tz))

        # Sort the list so the blue handle will be placed last if possible
        # sorted(found_objects, key=lambda obj: len(obj.mpe_object.primitives))
        found_objects.sort(key=lambda obj: len(obj.mpe_object.primitives))
        # sorted(objects_in_tzs, key=lambda obj_in_tz: len(obj_in_tz[0].mpe_object.primitives))
        objects_in_tzs.sort(key=lambda obj_in_tz: len(obj_in_tz[0].mpe_object.primitives))
        # If it's task 5 start with the biggest objects to get more coverage
        if userdata.yaml.task_type == Task.TASK_5:
            found_objects.reverse()
            objects_in_tzs.reverse()

        def get_pose(obj):
            return PointStamped(header, tzs_for[obj.mpe_object.id].target_position)

        rospy.logdebug('Objects in target zones: %s' % str(objects_in_tzs))
        rospy.logdebug('Target zones for objects: %s' % str(tzs_for))

        # If no object is in a target zone
        if not objects_in_tzs:
            plan = map(lambda obj: (obj, get_pose(obj)), found_objects)

        # If one object is in a target zone
        elif len(objects_in_tzs) == 1:
            found_objects.remove(objects_in_tzs[0][0])
            plan.append((objects_in_tzs[0][0], get_pose(objects_in_tzs[0][0])))
            plan += map(lambda obj: (obj, get_pose(obj)), found_objects)

        # If two objects are in a target zone
        elif len(objects_in_tzs) == 2:
            found_objects.remove(objects_in_tzs[0][0])
            found_objects.remove(objects_in_tzs[1][0])

            # If the one object is in the target zone of the other object in a target zone
            if objects_in_tzs[0][1].name == tzs_for[objects_in_tzs[1][0].mpe_object.id].name:
                # If the objects in target zone are in the target zone of the other object
                if objects_in_tzs[1][1].name == tzs_for[objects_in_tzs[0][0].mpe_object.id].name:
                    plan.append((objects_in_tzs[1][0],
                                 self._pose_near_target_zone(objects_in_tzs[1][1], header)))
                    plan.append((objects_in_tzs[0][0], get_pose(objects_in_tzs[0][0])))
                    plan.append((objects_in_tzs[1][0], get_pose(objects_in_tzs[1][0])))
                    plan += map(lambda obj: (obj, get_pose(obj)), found_objects)
                else:
                    plan.append((objects_in_tzs[0][0], get_pose(objects_in_tzs[0][0])))
                    plan.append((objects_in_tzs[1][0], get_pose(objects_in_tzs[1][0])))
                    plan += map(lambda obj: (obj, get_pose(obj)), found_objects)
            else:
                plan.append((objects_in_tzs[1][0], get_pose(objects_in_tzs[1][0])))
                plan.append((objects_in_tzs[0][0], get_pose(objects_in_tzs[0][0])))
                plan += map(lambda obj: (obj, get_pose(obj)), found_objects)

        # If all three objects are in a target zone
        elif len(objects_in_tzs) == 3:
            # Place the first object next to its targetzone
            plan.append((objects_in_tzs[0][0],
                         self._pose_near_target_zone(objects_in_tzs[0][1], header)))

            if objects_in_tzs[1][0].mpe_object.id == objects_in_tzs[0][1].expected_object:
                plan.append((objects_in_tzs[1][0], get_pose(objects_in_tzs[1][0])))
                plan.append((objects_in_tzs[2][0], get_pose(objects_in_tzs[2][0])))
            else:
                plan.append((objects_in_tzs[2][0], get_pose(objects_in_tzs[2][0])))
                plan.append((objects_in_tzs[1][0], get_pose(objects_in_tzs[1][0])))

            plan.append((objects_in_tzs[0][0], get_pose(objects_in_tzs[0][0])))

        userdata.clean_up_plan = plan
        return 'success'

    @staticmethod
    def _pose_near_target_zone(tgt, header):
        point = deepcopy(tgt.target_position)

        if point.x > 0:
            point.x += -tgt.max_distance - 0.05
        else:
            point.x += tgt.max_distance + 0.05

        if point.y > 0:
            point.y += -tgt.max_distance - 0.05
        else:
            point.y += tgt.max_distance + 0.05

        return PointStamped(header, point)