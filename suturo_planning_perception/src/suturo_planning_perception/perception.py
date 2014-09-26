import rospy
from suturo_perception_msgs.srv import *


def recognize_objects_of_interest(colors):
    rospy.wait_for_service('suturo/RecognizeOoI')
    try:
        reg_ooi = rospy.ServiceProxy('suturo/RecognizeOoI', RecognizeOoI)
        return reg_ooi(colors).foundObjects
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def get_gripper_perception(cuboid=True, pose_estimation=False, object_ids=[]):
    s = 'height,centroid,shape,color'
    if cuboid:
        s += ',cuboid'
    if pose_estimation:
        s += ',ModelPoseEstimation'
        if object_ids:
            s_ids = ''
            for object_id in object_ids:
                s_ids += str(object_id) + ','
            s_ids = '(' + s_ids[:-1] + ')'
            s += s_ids
    rospy.logdebug('Perception param: %s'%s)

    rospy.wait_for_service('suturo/GetGripper')
    try:
        perceived_objects = rospy.ServiceProxy('suturo/GetGripper', GetGripper)
        return perceived_objects(s).objects
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