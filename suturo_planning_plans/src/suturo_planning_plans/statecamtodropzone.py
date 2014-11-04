import smach
import rospy
import utils
import time
from suturo_planning_manipulation.manipulation import Manipulation
from tf.listener import TransformListener
import geometry_msgs
from geometry_msgs.msg import Pose
from math import pi
from suturo_planning_manipulation.mathemagie import rotate_quaternion


class CamToDropzone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanPoseReached', 'fail'],
                             input_keys=['yaml', 'scan_conveyor_pose'],
                             output_keys=['scan_conveyor_pose'])

    def execute(self, userdata):
        rospy.logdebug('CamToDropzone: Executing state CamToDropzone')

        listener = TransformListener()

        if utils.manipulation is None:
            utils.manipulation = Manipulation()

        # TODO: Exception schmeissen wenn abort_after vergangen ist
        abort_after = 15
        then = int(time.time())
        now = int(time.time())
        while now - then < abort_after and not listener.frameExists("drop_point"):
            rospy.loginfo("wait for drop_point frame")
            rospy.sleep(2.)
            now = int(time.time())
        then = int(time.time())
        now = int(time.time())
        while now - then < abort_after and not listener.frameExists("mdl_middle"):
            rospy.loginfo("wait for mdl_middle frame")
            rospy.sleep(2.)
            now = int(time.time())

        if len(userdata.scan_conveyor_pose) == 0:
            rospy.logdebug('CamToDropzone: No scanPose, calculate now...')
            p = utils.manipulation.scan_conveyor_pose()
            userdata.scan_conveyor_pose.append(p)
        else:
            rospy.logdebug('CamToDropzone: scanPose found!')
            p = userdata.scan_conveyor_pose[0]

        rospy.logdebug('CamToDropzone: Move arm to scan_conveyor_pose')

        for i in range(0, 3):
            if utils.manipulation.move_to(p, False):
                return 'scanPoseReached'
            else:
                if i == 2:
                    return 'fail'