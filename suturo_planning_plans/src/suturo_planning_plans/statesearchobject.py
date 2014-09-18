import smach
import rospy
import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from geometry_msgs.msg import PoseStamped


class SearchObject(smach.State):

    _next_scan = 0
    _current_position = 0
    _found_objects = []
    _recognized_objects = []
    _obj_colors = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['objectFound', 'noObjectsLeft'],
                             input_keys=['yaml', 'objects_found'],
                             output_keys=['object_to_perceive'])

    def execute(self, userdata):
        rospy.loginfo('Executing state SearchObject')

        if not self._obj_colors:
            for obj in userdata.yaml.objects:
                self._obj_colors.append(hex_to_color_msg(obj.color))

        # TODO find a way to take last pose
        # First check previously recognized objects
        # if self._recognized_objects:
        #     userdata.object_to_perceive = self._recognized_objects.pop(0)
        #     return 'objectFound'

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        scan_pose = 'scan_pose1'

        # Add the found objects
        if userdata.objects_found:
            self._found_objects = userdata.objects_found
            # Stop searching if all objects were found
            if len(self._found_objects) == len(userdata.yaml.objects):
                return 'noObjectsLeft'

            # After placing an object go back into the scan_pose1
            utils.manipulation.move_to(scan_pose)

        # take initial scan pose
        if self._next_scan == 0:
            rospy.loginfo('Take ' + scan_pose)
            utils.manipulation.move_to(scan_pose)

        # get the colors of the missing objects, assuming for now that every object has its own color
        colors = self._obj_colors
        for obj in self._found_objects:
            try:
                colors.remove(obj.color)
            except ValueError:
                pass

        # search for objects
        search_positions = [[0.3, 0.3, 0.0], [-0.3, 0.3, 0.0], [-0.3, -0.3, 0.0], [0.3, -0.3, 0.0]]
        num_of_scans = 12
        max_rad = 5.9
        rad_per_step = max_rad / num_of_scans

        for pos in range(self._current_position, len(search_positions)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = '/odom_combined'
            pose_stamped.header.stamp = rospy.get_rostime
            pose_stamped.pose.position.x = search_positions[pos][0]
            pose_stamped.pose.position.y = search_positions[pos][1]
            pose_stamped.pose.position.z = search_positions[pos][2]

            rospy.loginfo('Moving to %s'%str(pose_stamped))
            utils.manipulation.move_base(pose_stamped)

            for x in range(self._next_scan, num_of_scans):
                # skip turning arm on first scan
                if self._next_scan != 0:
                    rad = x * rad_per_step - 2.945
                    rospy.loginfo('Turning arm %s' %str(rad))
                    utils.manipulation.turn_arm(rad)

                rospy.sleep(1)
                self._next_scan += 1

                # look for objects
                rospy.loginfo('Colors: ' + str(colors))
                self._recognized_objects = perception.recognize_objects_of_interest(colors)
                rospy.loginfo('Found objects: ' + str(self._recognized_objects))

                if self._recognized_objects:  # check if an object was recognized
                    userdata.object_to_perceive = self._recognized_objects.pop(0)

                    # Might help with tf
                    rospy.loginfo('Sleep 1')
                    rospy.sleep(1)
                    rospy.loginfo('Sleep 2')
                    rospy.sleep(1)
                    rospy.loginfo('Sleep 3')
                    rospy.sleep(1)
                    rospy.loginfo('Sleep 4')
                    rospy.sleep(1)

                    return 'objectFound'

            self._next_scan = 0
            self._current_position += 1

            if pos != len(search_positions) - 1:
               rospy.loginfo('Take ' + scan_pose)
               utils.manipulation.move_to(scan_pose)

        return 'noObjectsLeft'