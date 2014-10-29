import smach
import rospy
import utils
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_msgs.msg import Task
from geometry_msgs.msg._PointStamped import PointStamped
import geometry_msgs.msg
from geometry_msgs.msg._Quaternion import Quaternion
from geometry_msgs.msg._Point import Point
import math
import roslib
import tf
from tf.listener import TransformListener
from suturo_planning_manipulation.transformer import Transformer

class CamToDropzone(smach.State):
    def __init__(self):
        self.tf = Transformer()
        self.__listener = TransformListener()
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state CamToDropzone')
        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        while not self.__listener.frameExists("drop_point"):
            print "wait for drop_point frame"
            rospy.sleep(5.)
        while not self.__listener.frameExists("mdl_middle"):
            print "wait for mdl_middle frame"
            rospy.sleep(5.)
        # TODO: 2. Dann in Manipulation iwas aufrufen, wo Pos ueber dem Band berechnet wird
        # TODO: 3. Arm ueber das Band bewegen damit Kamera subauf die Dropzone schaut
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.get_drop_point)

        m = Manipulation()
        scan_pose = m.scan_conveyor_pose(drop_point, 0.7854)
        #m.move_to("scan_conveyor_pose1")
        print scan_pose
        m.move_to(scan_pose)

        return 'success'

    def get_drop_point(self, msg):
        global drop_point
        drop_point = msg.conveyor_belt.drop_center_point
        pass