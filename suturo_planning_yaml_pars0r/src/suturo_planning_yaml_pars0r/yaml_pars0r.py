#!/usr/bin/env python
import yaml
import rospy
from yaml_exceptions import EmptyString
from yaml_exceptions import UnhandledValue
from yaml_exceptions import IllegalValue
from suturo_msgs.msg import Task
from suturo_msgs.msg import MastOfCam
from suturo_msgs.msg import Object
from suturo_msgs.msg import Shape
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3


class YamlPars0r:

    def __init__(self):
        self.pub = rospy.Publisher('suturo/yaml_pars0r', Task, queue_size=10, latch=True)
        rospy.init_node('yaml_pars0r_node', anonymous=True)
        self._subscriber = rospy.Subscriber("suturo/yaml_pars0r_input", String, self.get_input)

    def get_input(self, msg):
        self.publish(msg.data)

    @staticmethod
    def get_dict_value(dictionary, key):
        if (type(dictionary) is dict) and (key in dictionary):
            val = dictionary[key]
        else:
            val = ""
        rospy.loginfo("val: " + str(val))
        return val

    def publish(self, data):
        try:
            msg = self.parse_yaml(data)
            self.pub.publish(msg)
        except (EmptyString, UnhandledValue, IllegalValue, UnhandledValue, ValueError, TypeError) as e:
            rospy.logerr('publish: Could not parse yaml description: %s', e)
        except Exception as e:
            rospy.logerr('publish: Could not parse yaml description: %s', e)

    def parse_yaml(self, data):
        y = yaml.load(data)
        if y:
            f_description = self.get_dict_value(y, 'description')
            f_log_filename = self.get_dict_value(y, 'log_filename')
            f_mast_of_cam = self.get_dict_value(y, 'mast_of_cam')
            if f_mast_of_cam:
                f_base_pose = f_mast_of_cam['base_pose']
                f_pan_tilt_base = f_mast_of_cam['pan_tilt_base']
                f_speed_limit = f_mast_of_cam['speed_limit']
            else:
                f_base_pose = ""
                f_pan_tilt_base = ""
                f_speed_limit = ""
            try:
                f_objects = self.parse_objects(self.get_dict_value(y, 'objects'))
            except (UnhandledValue, IllegalValue, UnhandledValue, ValueError, TypeError) as e:
                rospy.logerr('parse_yaml: Exception while parsing objects: %s', e)
                raise e
            msg = Task(
                description=f_description,
                log_filename=f_log_filename,
                mast_of_cam=MastOfCam(
                    base_pose=f_base_pose,
                    pan_tilt_base=f_pan_tilt_base,
                    speed_limit=f_speed_limit
                ),
                objects=f_objects
            )
            return msg
        else:
            raise EmptyString('The given yaml description seems to be empty.')

    def parse_objects(self, objects):
        parsed_objects = []
        for obj in objects:
            f_name = obj
            rospy.loginfo('parse_objects: objects[obj]: %s', objects[obj])
            f_color = self.get_dict_value(objects[obj], 'color')
            f_description = self.get_dict_value(objects[obj], 'description')
            try:
                f_shapes = self.get_dict_value(objects[obj], 'shape')
                f_shapes = self.parse_shapes(f_shapes)
            except (UnhandledValue, IllegalValue, UnhandledValue, ValueError) as e:
                rospy.logerr('parse_objects: Exception %s while parsing shapes: %s', e, f_shapes)
                raise e
            f_material = self.get_dict_value(objects[obj], 'surface_material')
            if f_material == 'aluminium':
                f_material = Object.ALUMINIUM
            else:
                raise UnhandledValue('Unhandled surface_material: ' + str(f_material))
            rospy.loginfo('parse_objects: f_color: %s', f_color)
            rospy.loginfo('parse_objects: f_description: %s', f_description)
            rospy.loginfo('parse_objects: f_shapes: %s', f_shapes)
            rospy.loginfo('parse_objects: f_material: %s', f_material)
            parsed_objects.append(Object(
                name=f_name,
                color=f_color,
                description=f_description,
                shapes=f_shapes,
                surface_material=f_material
            ))
        return parsed_objects

    def parse_shapes(self, shapes):
        rospy.loginfo('parse_shapes: parsing shapes: %s', shapes)
        parsed_shapes = []
        for s in shapes:
            f_type = self.get_dict_value(s, 'type')
            try:
                if f_type == 'cylinder':
                    f_type = Shape.CYLINDER
                elif f_type == 'box':
                    f_type = Shape.BOX
                else:
                    raise UnhandledValue("Unhandled shape type: " + str(f_type))
                f_dimensions = [0.0] * 6
                f_pose = self.get_dict_value(s, 'pose')
                rospy.loginfo('parse_shapes: f_pose: %s', f_pose)
                try:
                    pose_msg = Twist(
                        linear=Vector3(
                            x=f_pose[0],
                            y=f_pose[1],
                            z=f_pose[2]
                        ),
                        angular=Vector3(
                            x=f_pose[3],
                            y=f_pose[4],
                            z=f_pose[5]
                        )
                    )
                except (IndexError, TypeError) as e:
                    raise IllegalValue('Apparently there are fields missing in the shape\'s pose: ' + str(f_pose))
                f_density = self.get_dict_value(s, 'density')
                if f_type == Shape.BOX:
                    f_size = self.get_dict_value(s, 'size')
                    f_dimensions[Shape.BOX_X] = f_size[0]
                    f_dimensions[Shape.BOX_Y] = f_size[1]
                    f_dimensions[Shape.BOX_Z] = f_size[2]
                elif f_type == Shape.CYLINDER:
                    f_dimensions[Shape.CYLINDER_LENGTH] = self.get_dict_value(s, 'length')
                    f_dimensions[Shape.CYLINDER_RADIUS] = self.get_dict_value(s, 'radius')
                else:
                    raise UnhandledValue("Unhandled shape type: " + str(f_type))
                parsed_shapes.append(Shape(
                    type=f_type,
                    dimensions=f_dimensions,
                    density=f_density,
                    pose=pose_msg
                ))
            except (ValueError) as e:
                rospy.logerr('parse_shapes: Exception %s while parsing shapes: %s', e, shapes)
                raise e
        return parsed_shapes

    def __del__(self):
        rospy.loginfo("Deleting yaml pars0r instance.")
        self._subscriber.unregister()
        rospy.signal_shutdown()