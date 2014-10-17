import rospy
import struct
import math
from std_msgs.msg import ColorRGBA
from suturo_perception_msgs.srv import Classifier
from geometry_msgs.msg import PointStamped


# Holds the manipulation object
manipulation = None
map = None


def classify_object(obj):
    rospy.wait_for_service('suturo/Classifier')
    try:
        classifier = rospy.ServiceProxy('suturo/Classifier', Classifier)
        return classifier(obj).classifiedObject
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def hex_to_color_msg(hex_str):
    rgb = struct.unpack('BBB', hex_str.decode('hex'))
    msg = ColorRGBA()
    msg.r = rgb[0]
    msg.g = rgb[1]
    msg.b = rgb[2]
    msg.a = 1
    return msg


def publish_collision_objects(objects):
    # pub = rospy.Publisher('collision_object', CollisionObject, queue_size=10)
    for obj in objects:
        manipulation.get_planning_scene().add_object(obj)
        # pub.publish(obj)


def get_object_to_move(objects):
    for obj in objects:
        # if obj.c_id == EurocObject.OBJECT:
        if obj.mpe_success:
            return obj

    return None


def get_valid_objects(objects):
    if objects is None:
        return []

    rospy.logdebug('Perceived: %s' % objects)

    # valid_objects = []
    # for obj in objects:
    #     if obj.c_cuboid_success and (obj.object.primitives[0].dimensions != (-1.0, -1.0, -1.0)):
    #         valid_objects.append(obj)
    return objects


def get_yaml_objects_nrs(yaml, object_id):
    nrs = []
    for i in range(0, len(yaml.objects)):
        if yaml.objects[i].name == object_id:
            nrs.append(i)
    rospy.logdebug('Object Nr for ' + object_id + ' ' + str(nrs))
    return nrs


# Check the centroids of obj against every object in objects
# Get the one that has the minimum euclidean distance to obj
# If the distance is below treshold, return the idx (beginning at 0).
# Otherwise, return None
def get_nearest_object_idx(obj, objects, treshold):
    # TODO PM
    min_distance = float("inf")
    i = 0
    min_distance_idx = None

    for o in objects:
        # Calculate Euclidean distance on geometry_msgs/Point
        dist = math.sqrt(math.pow(obj.c_centroid.x - o.c_centroid.x, 2) +
                         math.pow(obj.c_centroid.y - o.c_centroid.y, 2) +
                         math.pow(obj.c_centroid.z - o.c_centroid.z, 2))
        rospy.logdebug('Distance is ' + str(dist) + ' for idx ' + str(i))
        if dist < min_distance:
            min_distance = dist
            min_distance_idx = i
        i += 1
    if min_distance < treshold:
        rospy.logdebug('Best distance is ' + str(min_distance) + 'for idx' + str(min_distance_idx))
        return min_distance_idx
    return None


def centroid_to_odom_combined(euroc_object):
    camera_point = PointStamped()
    camera_point.header.stamp = rospy.Time(0)
    camera_point.header.frame_id = '/tdepth_pcl'
    camera_point.point = euroc_object.c_centroid
    odom_point = manipulation.transform_to(camera_point, '/odom_combined')
    euroc_object.c_centroid = odom_point.point