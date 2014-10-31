import smach
import rospy
import utils
from suturo_planning_manipulation.manipulation import Manipulation
from tf.listener import TransformListener


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
            rospy.sleep(2)

        # utils.manipulation.is_gripper_open()

        # TODO: Irgend ne Abbruchbedingung machen (nach x Sekunden)
        while not listener.frameExists("drop_point"):
            rospy.loginfo("wait for drop_point frame")
            rospy.sleep(2.)
        while not listener.frameExists("mdl_middle"):
            rospy.loginfo("wait for mdl_middle frame")
            rospy.sleep(2.)

        if len(userdata.scan_conveyor_pose) == 0:
            rospy.logdebug('CamToDropzone: No scanPose, calculate now...')
            p = utils.manipulation.scan_conveyor_pose()
            userdata.scan_conveyor_pose.append(p)
        else:
            rospy.logdebug('CamToDropzone: scanPose found!')
            p = userdata.scan_conveyor_pose[0]

        rospy.logdebug('CamToDropzone: Move arm to scan_conveyor_pose')
        utils.manipulation.move_to(p)

        return 'scanPoseReached'
        # if utils.manipulation.scan_conveyor_pose():
        #     rospy.logdebug('CamToDropzone: ScanPoseReached')
        #     return 'scanPoseReached'
        # else:
        #     rospy.loginfo('CamToDropzone: Cant reach ScanPose')
        #     return 'fail'