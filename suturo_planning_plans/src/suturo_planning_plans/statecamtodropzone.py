import smach
import rospy
import utils
import time
from suturo_planning_manipulation.manipulation import Manipulation
from tf.listener import TransformListener


class CamToDropzone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['scanPoseReached', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.logdebug('CamToDropzone: Executing state CamToDropzone')

        listener = TransformListener()

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        # TODO: Irgend ne Abbruchbedingung machen (nach x Sekunden)
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

        if utils.manipulation.scan_conveyor_pose():
            rospy.logdebug('CamToDropzone: ScanPoseReached')
            return 'scanPoseReached'
        else:
            rospy.loginfo('CamToDropzone: Cant reach ScanPose')
            return 'fail'