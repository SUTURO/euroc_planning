import smach
import rospy
import utils
from suturo_planning_manipulation.manipulation import Manipulation


class CamToDropzone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state CamToDropzone')
        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        # TODO: 2. Dann in Manipulation iwas aufrufen, wo Pos ueber dem Band berechnet wird
        # TODO: 3. Arm ueber das Band bewegen damit Kamera subauf die Dropzone schaut
        m = utils.manipulation
        m.move_to("scan_conveyor_pose1")

        return 'success'