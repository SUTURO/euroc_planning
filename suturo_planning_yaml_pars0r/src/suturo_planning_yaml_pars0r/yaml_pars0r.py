#!/usr/bin/env python
import yaml
import rospy
import re
import time
import signal
import sys
from yaml_exceptions import UnhandledValue
from suturo_msgs.msg import Task
from suturo_msgs.msg import MastOfCam
from suturo_msgs.msg import Object
from suturo_msgs.msg import Shape
from suturo_msgs.msg import Robot
from suturo_msgs.msg import Sensor
from suturo_msgs.msg import Camera
from suturo_msgs.msg import TargetZone
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point


class YamlPars0r:

    def __init__(self):
        self.pub = rospy.Publisher('suturo/yaml_pars0r', Task, queue_size=10, latch=True)
        rospy.init_node('yaml_pars0r_node', anonymous=True, log_level=rospy.INFO)
        self._subscriber = rospy.Subscriber("suturo/yaml_pars0r_input", String, self.get_input)
        rospy.loginfo('Initialised YAML parser.')

    def get_input(self, msg):
        rospy.loginfo('Received input')
        rospy.logdebug('get_input: ' + str(msg))
        self.publish(msg.data)

    @staticmethod
    def get_dict_value(dictionary, key, val_type=None):
        if (type(dictionary) is dict) and (key in dictionary):
            val = dictionary[key]
        elif val_type is float:
            val = float('inf')
        elif val_type is list:
            val = []
        else:
            val = None
        rospy.logdebug("key: " + str(key) + ", val: " + str(val))
        return val

    def publish(self, data):
        try:
            msg = self.parse_yaml(data)
            rospy.logdebug('publish: Publishing message: ' + str(msg))
            rospy.loginfo('Publishing description for task \'%s\'', msg.task_name)
            self.pub.publish(msg)
        except UnhandledValue as e:
            rospy.logerr('publish: Could not parse yaml description: %s: %s', type(e), e)
        except Exception as e:
            rospy.logerr('publish: Could not parse yaml description: %s: %s', type(e), e)

    @staticmethod
    def parse_yaml(data):
        rospy.loginfo('Parsing...')
        y = yaml.load(data)
        if y is not None:
            f_description = YamlPars0r.get_dict_value(y, 'description')
            f_log_filename = YamlPars0r.get_dict_value(y, 'log_filename')
            f_mast_of_cam = YamlPars0r.get_dict_value(y, 'mast_of_cam')
            f_base_pose = YamlPars0r.parse_twist(YamlPars0r.get_dict_value(f_mast_of_cam, 'base_pose'))
            f_pan_tilt_base = YamlPars0r.parse_twist(YamlPars0r.get_dict_value(f_mast_of_cam, 'pan_tilt_base'))
            f_speed_limit = YamlPars0r.get_dict_value(f_mast_of_cam, 'speed_limit')
            mast_of_cam_msg = MastOfCam(
                base_pose=f_base_pose,
                pan_tilt_base=f_pan_tilt_base,
                speed_limit=f_speed_limit
            )
            f_objects = YamlPars0r.parse_objects(YamlPars0r.get_dict_value(y, 'objects'))
            f_robot = YamlPars0r.get_dict_value(y, 'robot')
            gripper_pose_msg = YamlPars0r.parse_twist(YamlPars0r.get_dict_value(f_robot, 'gripper_pose'))
            f_gripper_speed_limit = YamlPars0r.get_dict_value(f_robot, 'gripper_speed_limit', float)
            gripper_tcp_msg = YamlPars0r.parse_twist(YamlPars0r.get_dict_value(f_robot, 'gripper_tcp'))
            robot_pose_msg = YamlPars0r.parse_twist(YamlPars0r.get_dict_value(f_robot, 'pose'))
            f_robot_speed_limit = YamlPars0r.get_dict_value(f_robot, 'speed_limit', list)
            rospy.logdebug('f_robot_speed_limit: ' + str(f_robot_speed_limit))
            robot_msg = Robot()
            robot_msg.gripper_pose = gripper_pose_msg
            robot_msg.gripper_speed_limit = f_gripper_speed_limit
            robot_msg.gripper_tcp = gripper_tcp_msg
            robot_msg.pose = robot_pose_msg
            robot_msg.speed_limit = f_robot_speed_limit
            f_sensors = YamlPars0r.get_dict_value(y, 'sensors')
            sensors_msg = YamlPars0r.parse_sensors(f_sensors)
            f_target_zones = YamlPars0r.get_dict_value(y, 'target_zones')
            target_zones_msg = YamlPars0r.parse_target_zones(f_target_zones)
            f_task_name = YamlPars0r.get_dict_value(y, 'task_name')
            msg = Task(
                description=f_description,
                log_filename=f_log_filename,
                mast_of_cam=mast_of_cam_msg,
                objects=f_objects,
                robot=robot_msg,
                sensors=sensors_msg,
                target_zones=target_zones_msg,
                task_type=YamlPars0r.parse_task_type(f_task_name),
                task_name=f_task_name,
                time_limit=YamlPars0r.get_dict_value(y, 'time_limit', float),
                two_axes_table_speed_limit=YamlPars0r.get_dict_value(YamlPars0r.get_dict_value(y, 'two_axes_table'), 'speed_limit', list)
            )
            return msg
        else:
            return Task()

    @staticmethod
    def parse_task_type(task):
        if re.match('task 1', task) is not None:
            return Task.TASK_1
        elif re.match('task 2', task) is not None:
            return Task.TASK_2
        elif re.match('task 3', task) is not None:
            return Task.TASK_3
        elif re.match('task 4', task) is not None:
            return Task.TASK_4
        elif re.match('task 5', task) is not None:
            return Task.TASK_5
        elif re.match('task 6', task) is not None:
            return Task.TASK_6
        else:
            raise UnhandledValue('parse_task_type: Unhandled task type: ' + task)

    @staticmethod
    def parse_target_zones(zones):
        parsed_target_zones = []
        if zones is None:
            return parsed_target_zones
        for z in zones:
            zone_msg = TargetZone()
            zone_msg.name = z
            f_expected_object = YamlPars0r.get_dict_value(zones[z], 'expected_object')
            if f_expected_object is not None:
                zone_msg.expected_object = f_expected_object
            zone_msg.max_distance = YamlPars0r.get_dict_value(zones[z], 'max_distance')
            f_target_position = YamlPars0r.get_dict_value(zones[z], 'target_position')
            target_position_msg = Point()
            try:
                target_position_msg.x = f_target_position[0]
            except IndexError:
                rospy.loginfo('parse_target_zones: x value in target_position is not set.')
            try:
                target_position_msg.y = f_target_position[1]
            except IndexError:
                rospy.loginfo('parse_target_zones: y value in target_position is not set.')
            try:
                target_position_msg.z = f_target_position[2]
            except IndexError:
                rospy.loginfo('parse_target_zones: z value in target_position is not set.')
            zone_msg.target_position = target_position_msg
            parsed_target_zones.append(zone_msg)
        return parsed_target_zones

    @staticmethod
    def parse_sensors(sensors):
        parsed_sensors = []
        if sensors is None:
            return parsed_sensors
        for s in sensors:
            f_name = s
            rospy.logdebug('parse_sensors: sensors[s]: %s', sensors[s])
            if f_name == 'scene_depth_cam':
                f_sensor_type = Sensor.CAMERA
                f_cam_type = Camera.SCENE_DEPTH_CAM
            elif f_name == 'scene_rgb_cam':
                f_sensor_type = Sensor.CAMERA
                f_cam_type = Camera.SCENE_RGB_CAM
            elif f_name == 'tcp_depth_cam':
                f_sensor_type = Sensor.CAMERA
                f_cam_type = Camera.TCP_DEPTH_CAM
            elif f_name == 'tcp_rgb_cam':
                f_sensor_type = Sensor.CAMERA
                f_cam_type = Camera.TCP_RGB_CAM
            else:
                raise UnhandledValue('Unhandled sensor type: ' + str(f_name))
            sensor_msg = Sensor(sensor_type=f_sensor_type)
            if f_sensor_type == Sensor.CAMERA:
                camera_msg = Camera(camera_type=f_cam_type)
                f_horizontal_fov = YamlPars0r.get_dict_value(sensors[s]['camera'], 'horizontal_fov', float)
                if f_horizontal_fov is not None:
                    camera_msg.horizontal_fov = f_horizontal_fov
                f_image = YamlPars0r.get_dict_value(sensors[s]['camera'], 'image')
                f_image_width = YamlPars0r.get_dict_value(f_image, 'width', float)
                f_image_height = YamlPars0r.get_dict_value(f_image, 'height', float)
                if f_image_width is not None:
                    camera_msg.image_width = f_image_width
                if f_image_height is not None:
                    camera_msg.image_height = f_image_height
                sensor_msg.camera = camera_msg
            else:
                raise UnhandledValue('Unhandled sensor type: ' + str(f_sensor_type))
            f_relative_pose = YamlPars0r.get_dict_value(sensors[s], 'relative_pose')
            relative_pose_msg = YamlPars0r.parse_twist_stamped(f_relative_pose)
            f_pose = YamlPars0r.get_dict_value(sensors[s], 'pose')
            pose_msg = YamlPars0r.parse_twist(f_pose)
            f_update_rate = YamlPars0r.get_dict_value(sensors[s], 'update_rate', float)
            if relative_pose_msg is not None:
                sensor_msg.relative_pose = relative_pose_msg
            if pose_msg is not None:
                sensor_msg.pose = pose_msg
            if f_update_rate is not None:
                sensor_msg.update_rate = f_update_rate
            parsed_sensors.append(sensor_msg)
        return parsed_sensors

    @staticmethod
    def parse_twist_stamped(twist_stamped):
        rospy.logdebug('parse_twist_stamped: parsing ' + str(twist_stamped))
        f_from = YamlPars0r.get_dict_value(twist_stamped, 'from')
        f_pose = YamlPars0r.get_dict_value(twist_stamped, 'pose')
        twist_msg = YamlPars0r.parse_twist(f_pose)
        header_msg = Header()
        if f_from is not None:
            header_msg.frame_id = f_from
        twist_stamped_msg = TwistStamped(
            header=header_msg
        )
        if twist_msg is not None:
            twist_stamped_msg.twist = twist_msg
        return twist_stamped_msg

    @staticmethod
    def parse_twist(twist):
        rospy.logdebug('parse_twist: parsing ' + str(twist))
        f_linear = Vector3(float('inf'), float('inf'), float('inf'))
        f_angular = Vector3(float('inf'), float('inf'), float('inf'))
        if twist is not None:
            try:
                f_linear.x = twist[0]
            except IndexError:
                rospy.loginfo('parse_twist: x value in gripper pose is not set.')
            try:
                f_linear.y = twist[1]
            except IndexError:
                rospy.loginfo('parse_twist: y value in gripper pose is not set.')
            try:
                f_linear.z = twist[2]
            except IndexError:
                rospy.loginfo('parse_twist: z value in gripper pose is not set.')
            try:
                f_angular.x = twist[3]
            except IndexError:
                rospy.loginfo('parse_twist: Roll value in gripper pose is not set.')
            try:
                f_angular.y = twist[4]
            except IndexError:
                rospy.loginfo('parse_twist: Pitch value in gripper pose is not set.')
            try:
                f_angular.z = twist[5]
            except IndexError:
                rospy.loginfo('parse_twist: Yaw value in gripper pose is not set.')
        return Twist(
            linear=f_linear,
            angular=f_angular
        )

    @staticmethod
    def parse_objects(objects):
        parsed_objects = []
        if objects is None:
            return parsed_objects
        for obj in objects:
            f_name = obj
            rospy.logdebug('parse_objects: objects[obj]: %s', objects[obj])
            f_color = YamlPars0r.get_dict_value(objects[obj], 'color')
            f_description = YamlPars0r.get_dict_value(objects[obj], 'description')
            try:
                f_shapes = YamlPars0r.get_dict_value(objects[obj], 'shape')
                f_shapes = YamlPars0r.parse_shapes(f_shapes)
            except UnhandledValue as e:
                rospy.logerr('parse_objects: Exception %s while parsing shapes: %s', e, f_shapes)
                raise e
            f_material = YamlPars0r.get_dict_value(objects[obj], 'surface_material')
            if f_material == 'aluminium':
                f_material = Object.ALUMINIUM
            elif f_material is None:
                pass
            else:
                raise UnhandledValue('Unhandled surface_material: ' + str(f_material))
            rospy.logdebug('parse_objects: f_color: %s', f_color)
            rospy.logdebug('parse_objects: f_description: %s', f_description)
            rospy.logdebug('parse_objects: f_shapes: %s', f_shapes)
            rospy.logdebug('parse_objects: f_material: %s', f_material)
            obj = Object(shapes=f_shapes)
            if f_name is not None:
                obj.name = f_name
            if f_color is not None:
                obj.color = f_color
            if f_description is not None:
                obj.description = f_description
            if f_material is not None:
                obj.surface_material = f_material
            parsed_objects.append(obj)
        return parsed_objects

    @staticmethod
    def parse_shapes(shapes):
        rospy.logdebug('parse_shapes: parsing shapes: %s', shapes)
        parsed_shapes = []
        if shapes is None:
            return parsed_shapes
        for s in shapes:
            f_type = YamlPars0r.get_dict_value(s, 'type')
            if f_type == 'cylinder':
                f_type = Shape.CYLINDER
            elif f_type == 'box':
                f_type = Shape.BOX
            else:
                raise UnhandledValue("Unhandled shape type: " + str(f_type))
            f_pose = YamlPars0r.get_dict_value(s, 'pose')
            rospy.logdebug('parse_shapes: f_pose: %s', f_pose)
            pose_msg = YamlPars0r.parse_twist(f_pose)
            f_density = YamlPars0r.get_dict_value(s, 'density', float)
            f_dimensions = [float('inf')] * 6
            rospy.logdebug('parse_shapes: f_type: %s', f_type)
            if f_type == Shape.BOX:
                f_size = YamlPars0r.get_dict_value(s, 'size')
                try:
                    f_dimensions[Shape.BOX_X] = f_size[0]
                except IndexError:
                    rospy.loginfo('x value in dimensions is not set.')
                try:
                    f_dimensions[Shape.BOX_Y] = f_size[1]
                except IndexError:
                    rospy.loginfo('Y value in dimensions is not set.')
                try:
                    f_dimensions[Shape.BOX_Z] = f_size[2]
                except IndexError:
                    rospy.loginfo('z value in dimensions is not set.')
            elif f_type == Shape.CYLINDER:
                f_length = YamlPars0r.get_dict_value(s, 'length', float)
                f_radius = YamlPars0r.get_dict_value(s, 'radius', float)
                if f_length is not None:
                    f_dimensions[Shape.CYLINDER_LENGTH] = f_length
                if f_radius is not None:
                    f_dimensions[Shape.CYLINDER_RADIUS] = f_radius
            else:
                raise UnhandledValue("Unhandled shape type: " + str(f_type))
            shape = Shape(
                dimensions=f_dimensions,
                pose=pose_msg,
                shape_type=f_type,
                density=f_density
            )
            parsed_shapes.append(shape)
        return parsed_shapes

    def kill(self, sig, frame):
        rospy.loginfo('Caught ctrl-c. Shutting down.')
        self._subscriber.unregister()
        rospy.signal_shutdown("Shutting down YAML pars0r")
        sys.exit()


def main():
    y = YamlPars0r()
    signal.signal(signal.SIGINT, lambda sig, frame: y.kill(sig, frame))
    while True:
        time.sleep(0.05)

if __name__ == "__main__":
    main()