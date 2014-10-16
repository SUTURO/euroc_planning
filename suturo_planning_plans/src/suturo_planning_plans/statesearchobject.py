import smach
import rospy
from geometry_msgs.msg import PoseStamped

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception


class SearchObject(smach.State):
    _next_scan = 0
    _current_position = 0
    _found_objects = []
    _recognized_objects = []
    _obj_colors = []
    _missing_objects = None
    _last_joint_state = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsLeft'],
                             input_keys=['yaml', 'perceived_objects', 'task', 'fitted_object', 'enable_movement'],
                             output_keys=['object_to_perceive', 'objects_found'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        if self._missing_objects is None:
            self._missing_objects = []
            for obj in userdata.yaml.objects:
                self._missing_objects.append(obj.name)

        if not self._obj_colors:
            for obj in userdata.yaml.objects:
                self._obj_colors.append(hex_to_color_msg(obj.color))

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        scan_pose = 'scan_pose1'

        # Add the found objects
        if not userdata.fitted_object is None:
            for obj in [userdata.fitted_object]:
                # Check if the object was already found
                if [x for x in self._found_objects if x.mpe_object.id == obj.mpe_object.id]:
                    rospy.loginfo('Object %s was already found.' % obj.mpe_object.id)
                else:
                    self._found_objects.append(obj)
                    if obj.mpe_object.id in self._missing_objects:
                        self._missing_objects.remove(obj.mpe_object.id)
        userdata.objects_found = self._found_objects

        # Check if all objects were found
        if not self._missing_objects:
            return 'noObjectsLeft'

        # After focusing an object go back into the scan_pose1
        if self._last_joint_state is None:
            utils.manipulation.move_to(scan_pose)
        else:
            if userdata.enable_movement:
                utils.manipulation.move_arm_and_base_to(self._last_joint_state)
            else:
                utils.manipulation.move_to(self._last_joint_state)

        # take initial scan pose
        # if self._next_scan == 0:
        #     rospy.loginfo('Take %s' % scan_pose)
        #     utils.manipulation.move_to(scan_pose)

        colors = self._obj_colors

        # search for objects
        if userdata.task == 'task3':
            search_positions = [[0.3, 0.3, 0.0], [-0.3, 0.3, 0.0], [-0.3, -0.3, 0.0], [0.3, -0.3, 0.0]]
        else:
            search_positions = [[0.0, 0.0, 0.0]]
        num_of_scans = 8
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans

        for pos in range(self._current_position, len(search_positions)):

            if userdata.enable_movement:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = '/odom_combined'
                pose_stamped.header.stamp = rospy.get_rostime()
                pose_stamped.pose.position.x = search_positions[pos][0]
                pose_stamped.pose.position.y = search_positions[pos][1]
                pose_stamped.pose.position.z = search_positions[pos][2]

                rospy.loginfo('Moving to %s' % str(pose_stamped))
                utils.manipulation.move_base(pose_stamped)
                rospy.logdebug('Moved')

            for x in range(self._next_scan, num_of_scans):

                rad = x * rad_per_step - 2.945
                rospy.loginfo('Turning arm %s' % str(rad))
                utils.manipulation.turn_arm(rad)

                self._next_scan += 1

                # look for objects
                rospy.loginfo('Colors: %s' % str(colors))
                self._recognized_objects = perception.recognize_objects_of_interest(colors)
                rospy.loginfo('Found objects: %s' % str(self._recognized_objects))

                if self._recognized_objects:  # check if an object was recognized
                    userdata.object_to_perceive = self._recognized_objects.pop(0)

                    # Might help with tf
                    rospy.logdebug('Wait for tf')
                    rospy.sleep(5)
                    if userdata.enable_movement:
                        self._last_joint_state = utils.manipulation.get_arm_base_move_group().get_current_joint_values()
                    else:
                        self._last_joint_state = utils.manipulation.get_arm_move_group().get_current_joint_values()

                    return 'objectFound'

            self._next_scan = 0
            self._current_position += 1

            if pos != len(search_positions) - 1:
                rospy.loginfo('Take %s' % scan_pose)
                utils.manipulation.move_to(scan_pose)

        return 'noObjectsLeft'