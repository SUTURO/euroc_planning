import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation import manipulation_service

import utils
from utils import hex_to_color_msg
from suturo_planning_perception import perception


class Turn(smach.State):
    _next_scan = 0
    _current_position = 0
    _recognized_objects = []
    _obj_colors = []
    _last_joint_state = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsFound', 'simStopped'],
                             input_keys=['yaml', 'task', 'objects_found', 'enable_movement'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        if not self._obj_colors:
            for obj in userdata.yaml.objects:
                self._obj_colors.append(hex_to_color_msg(obj.color))

        scan_pose = 'scan_pose1'

        # After focusing an object go back into the scan_pose1
        if self._last_joint_state is None:
            utils.manipulation.move_to(scan_pose)
        else:
            if userdata.enable_movement:
                utils.manipulation.move_arm_and_base_to(self._last_joint_state)
            else:
                utils.manipulation.move_to(self._last_joint_state)

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
                rospy.loginfo('Colors: %s' % str(self._obj_colors))
                self._recognized_objects = perception.recognize_objects_of_interest(self._obj_colors)
                rospy.loginfo('Found objects: %s' % str(self._recognized_objects))

                if self._recognized_objects:
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

        return 'noObjectsFound'