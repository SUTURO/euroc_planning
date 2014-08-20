import rospy
from suturo_perception_msgs.srv import *


def recognize_objects_of_interest(colors):
    rospy.wait_for_service('suturo/RecognizeOoI')
    try:
        reg_ooi = rospy.ServiceProxy('suturo/RecognizeOoI', RecognizeOoI)
        return reg_ooi(colors).foundObjects
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_gripper_perception():
    rospy.wait_for_service('suturo/GetGripper')
    try:
        perceived_objects = rospy.ServiceProxy('suturo/GetGripper', GetGripper)
        return perceived_objects().objects
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_scene_perception():
    rospy.wait_for_service('suturo/GetScene')
    try:
        perceived_objects = rospy.ServiceProxy('suturo/GetScene', GetScene)
        return perceived_objects().objects
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_table():
    1