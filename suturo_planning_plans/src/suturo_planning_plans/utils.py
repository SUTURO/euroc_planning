import rospy
import struct
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import CollisionObject
from suturo_perception_msgs.msg import EurocObject


def hex_to_color_msg(hex_str):
    rgb = struct.unpack('BBB', hex_str.decode('hex'))
    msg = ColorRGBA()
    msg.r = rgb[0]
    msg.g = rgb[1]
    msg.b = rgb[2]
    msg.a = 1
    return msg


def publish_collision_objects(objects):
    pub = rospy.Publisher('collision_object', CollisionObject, queue_size=10)
    for obj in objects:
        pub.publish(obj)


def get_object_to_move(objects):
    for obj in objects:
        # if obj.c_id == EurocObject.OBJECT:
        if obj.object.primitives[0].dimensions != (-1.0, -1.0, -1.0):
            return obj

    return None


def get_valid_objects(objects):
    if objects is None:
        return []

    valid_objects = []
    for obj in objects:
        if obj.object.primitives[0].dimensions != (-1.0, -1.0, -1.0):
            valid_objects.append(obj)
    return valid_objects
