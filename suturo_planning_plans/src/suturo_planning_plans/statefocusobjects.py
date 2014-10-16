import smach
import rospy
from math import pi
import time
from suturo_planning_plans import utils
from suturo_planning_manipulation import calc_grasp_position
from geometry_msgs.msg import PointStamped


class FocusObjects(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml', 'objects_to_focus', 'enable_movement'],
                             output_keys=['focused_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FocusObject')

        userdata.focused_object = None

        camera_point = PointStamped()
        camera_point.header.stamp = rospy.Time(0)
        camera_point.header.frame_id = '/tdepth_pcl'
        camera_point.point = userdata.objects_to_focus[0].c_centroid
        odom_point = utils.manipulation.transform_to(camera_point, '/odom_combined')
        odom_point.point.z -= 0.05

        poses = calc_grasp_position.make_scan_pose(odom_point.point, 0.6, pi / 4.0)
        for pose in poses:
            if utils.manipulation.move_to(pose):
                userdata.focused_object = userdata.objects_to_focus[0]

                rospy.logdebug('Wait for clock')
                time.sleep(1)

                rospy.logdebug('Wait for tf again.')
                #start = time.time()
                rospy.sleep(1)
                rospy.logdebug('1: %s' % str(rospy.get_rostime()))
                rospy.sleep(1)
                rospy.logdebug('2: %s' % str(rospy.get_rostime()))
                rospy.sleep(1)
                rospy.logdebug('3: %s' % str(rospy.get_rostime()))
                rospy.sleep(1)
                rospy.logdebug('4: %s' % str(rospy.get_rostime()))
                rospy.sleep(1)
                rospy.logdebug('5: %s Continue.' % str(rospy.get_rostime()))



                a = 12 - 45
                a

                return 'success'

        return 'fail'