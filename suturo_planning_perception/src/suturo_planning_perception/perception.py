import rospy
from suturo_perception_msgs import *


def recognize_objects_of_interest():
    rospy.wait_for_service('suturo/RecognizeOoI')
    try:
        reg_ooi = rospy.ServiceProxy('suturo/RecognizeOoI', 0)  # import real service type
        return reg_ooi()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_gripper_perception():
    1


def get_scene_perception():
    1


def get_table():
    1