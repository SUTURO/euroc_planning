import rospy
import struct
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import CollisionObject
from suturo_perception_msgs.srv import Classifier


# Holds the manipulation object
manipulation = None


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

    valid_objects = []
    for obj in objects:
        if obj.c_cuboid_success and (obj.object.primitives[0].dimensions != (-1.0, -1.0, -1.0)):
            valid_objects.append(obj)
    return valid_objects


def get_yaml_objects_nrs(yaml, object_id):
    nrs = []
    for i in range(0, len(yaml.objects)):
        if yaml.objects[i].name == object_id:
            nrs.append(i)
    rospy.logdebug('Object Nr for ' + object_id + ' ' + str(nrs))
    return nrs