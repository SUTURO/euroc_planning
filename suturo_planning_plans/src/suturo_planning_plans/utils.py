import rospy
import struct
from std_msgs.msg import ColorRGBA
from moveit_msgs.msg import CollisionObject
from suturo_perception_msgs.srv import Classifier


# Holds the manipulation object
manipulation = None


def classify_object(object):
    rospy.wait_for_service('suturo/Classifier')
    try:
        classifier = rospy.ServiceProxy('suturo/Classifier', Classifier)
        return classifier(object).classifiedObject
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
    pub = rospy.Publisher('collision_object', CollisionObject, queue_size=10)
    for obj in objects:
        pub.publish(obj)


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


def match_object(obj, yaml):
    description = get_yaml_for_object(obj, yaml)
    obj.color = hex_to_color_msg(description.color)
    obj.object.id = description.name
    return description


def get_yaml_for_object(obj, yaml):
    diff = 9999  # a high value
    best_fit = None
    object_volume = get_volume_euroc_object(obj)
    for yaml_object in yaml.objects:
        new_diff = abs(object_volume - get_volume_yaml_object(yaml_object))
        if new_diff < diff:
            diff = new_diff
            best_fit = yaml_object
    return best_fit


def get_volume_euroc_object(obj):
    volume = 0
    for primitive in obj.object.primitives:
        volume += get_volume_primitive(primitive, primitive.type)
    return volume


def get_volume_yaml_object(obj):
    volume = 0
    for shape in obj.primitives:
        volume += get_volume_primitive(shape, shape.type)
    return volume


def get_volume_primitive(primitive, primitive_type):
    get_volume = {1: volume_box, 2: volume_sphere, 3: volume_cylinder, 3: volume_cone}
    return get_volume.get(primitive_type)(primitive)


def volume_box(primitive):
    return primitive.dimensions[0] * primitive.dimensions[1] * primitive.dimensions[2]


def volume_sphere(primitive):
    return primitive.dimensions[0] ** 3


def volume_cylinder(primitive):
    return primitive.dimensions[0] * ((primitive.dimensions[1] * 2) ** 2)


def volume_cone(primitive):
    return primitive.dimensions[0] * ((primitive.dimensions[1] * 2) ** 2)