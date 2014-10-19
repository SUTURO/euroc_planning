from copy import deepcopy
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Vector3 import Vector3
from math import sqrt
from moveit_msgs.msg._CollisionObject import CollisionObject
import smach
import rospy
from suturo_msgs.msg import Task
from suturo_planning_manipulation.mathemagie import magnitude, subtract_point
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position, get_pre_grasp
import utils


class CamToDropzone(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state CamToDropzone')

        # TODO: 1. Foerderband in PS haun, wenns noch nicht drin ist.
        # Jetzt in nem eigenen State

        # TODO: 2. Dann in Manipulation iwas aufrufen, wo Pos ueber dem Band berechnet wird
        # TODO: 3. Arm ueber das Band bewegen damit Kamera auf die Dropzone schaut

        return 'fail'