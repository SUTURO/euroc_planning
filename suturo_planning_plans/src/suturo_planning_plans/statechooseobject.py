from copy import deepcopy
import smach
import rospy
from geometry_msgs.msg import PointStamped
from suturo_planning_plans import utils
from std_msgs.msg import Header


class ChooseObject(smach.State):

    _ctr = 0

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectChosen', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found', 'clean_up_plan'],
                             output_keys=['object_to_move', 'place_position'])

    def execute(self, userdata):
        rospy.loginfo('Executing state ChooseObject')

        if len(userdata.clean_up_plan) > self._ctr:
            action = userdata.clean_up_plan[self._ctr]
            userdata.object_to_move = action[0]
            self._ctr += 1
        else:
            self._ctr = 0
            return 'noObjectsLeft'

        rospy.loginfo('Placing object on location %s' % action[1])
        userdata.place_position = action[1]

        return 'objectChosen'


class CleanUpPlan(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['clean_up_plan'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PlanCleanUp')

        header = Header()
        header.frame_id = '/odom_combined'
        plan = []
        tzs_for = {tz.expected_object: tz for tz in userdata.yaml.target_zones}
        found_objects = userdata.objects_found

        # Get the objects that are in an others objects target zone
        objects_in_tzs = []
        for obj in found_objects:
            tz = utils.in_target_zone(obj, userdata.yaml)
            if not tz is None and tz.expected_object != obj.mpe_object.id:
                objects_in_tzs.append((obj, tz))

        # Sort the list so the blue handle will be placed last if possible
        sorted(objects_in_tzs, key=lambda obj_in_tz: len(obj_in_tz[0].mpe_object.primitives))

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