import smach
import rospy
from suturo_msgs.msg import Task
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Quaternion import Quaternion
from geometry_msgs.msg._Point import Point
import math
import utils
from suturo_planning_manipulation.manipulation import Manipulation


class Task6Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'],
                             input_keys=['yaml'],
                             output_keys=[])

    def execute(self, userdata):
        rospy.loginfo('Executing state CamToDropzone')

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        # 1. Foerderband in PS haun, wenns noch nicht drin ist.
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.addConveyorBelt)
        # 1: Dropzone Punkt ueber Vektor dp finden
        # 2: Breite des Bandes ueber 2*drop_deviation[1] bestimmen
        # 3: Hoehe des Bandes ueber drop_center_point[2] (z wert) + Banddicke (0.01m) holen
        # 4: Lange des Bandes mit mdl Vektor bestimmen + 0.10m fuer Laenge hinter dem dp Vektor
        # TODO 5: Orientierung de Bandes mit mdl Vektor bestimmen

        return 'success'

    def addConveyorBelt(self, msg):
        print "addConveyorBelt callback"
        # print msg
        # zu 1
        drop_point = msg.conveyor_belt.drop_center_point
        print drop_point
        # zu 2
        conveyor_belt_width = msg.conveyor_belt.drop_deviation.y * 2
        print conveyor_belt_width
        # zu 3
        conveyor_belt_height = drop_point.z
        print conveyor_belt_height
        # zu 4
        conveyor_belt_length = msg.conveyor_belt.move_direction_and_length.y
        print conveyor_belt_length

        pose = PoseStamped()
        pose.header.frame_id = "/odom_combined"
        # h w d
        # pose.pose.position = Point(drop_point.x,
        # (drop_point.y - abs(conveyor_belt_length / 2)),
        # drop_point.z - abs(conveyor_belt_height / 2))
        # pose.pose.orientation = Quaternion(0, 0, 0, 1)

        l = math.sqrt(
            msg.conveyor_belt.move_direction_and_length.x * msg.conveyor_belt.move_direction_and_length.x
            + conveyor_belt_length * conveyor_belt_length)
        alpha = math.pi / 2 - math.asin(conveyor_belt_length / l)
        wx = conveyor_belt_width * math.cos(alpha)
        wy = conveyor_belt_width * math.sin(alpha)
        w = wx + wy

        pose.pose.position = Point(drop_point.x,
                                   (drop_point.y - abs(conveyor_belt_length / 2)),
                                   drop_point.z - abs(conveyor_belt_height / 2))
        pose.pose.orientation = Quaternion(0, 0, 0, 1)

        print pose

        cb_size = [w * 2, l, conveyor_belt_height]

        co = utils.manipulation.get_planning_scene().make_box("conveyor_belt", pose, cb_size)
        utils.manipulation.get_planning_scene().add_object(co)