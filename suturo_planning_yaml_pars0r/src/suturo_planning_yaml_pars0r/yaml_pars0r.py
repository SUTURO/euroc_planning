#!/usr/bin/env python
from jinja2.nodes import Concat
import yaml
from geometry_msgs.msg._Quaternion import Quaternion
import time
import rospy
import re
import sys
from copy import copy
from tf.transformations import quaternion_inverse, quaternion_from_euler
from yaml_exceptions import UnhandledValue
from suturo_msgs.msg import Task
from suturo_msgs.msg import MastOfCam
from suturo_msgs.msg import Object
from suturo_msgs.msg import Robot
from suturo_msgs.msg import Sensor
from suturo_msgs.msg import Camera
from suturo_msgs.msg import TargetZone
from suturo_msgs.msg import ConveyorBelt
from suturo_msgs.msg import PuzzlePart
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


class YamlPars0r:

    def __init__(self):
        self.pub = rospy.Publisher('suturo/startup/yaml_pars0r', Task, queue_size=10, latch=True)
        #rospy.init_node('yaml_pars0r_node', anonymous=True, log_level=rospy.INFO)
        self._subscriber = rospy.Subscriber("suturo/startup/yaml_pars0r_input", String, self.get_input)
        rospy.loginfo('Initialised YAML parser.')

    def get_input(self, msg):
        rospy.loginfo('Received input.')
        rospy.logdebug('get_input: ' + str(msg))
        self.parse_and_publish(msg.data)

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

    def parse_and_publish(self, data):
        rospy.loginfo('parse_and_publish')
        try:
            msg = self.parse_yaml(data)
            rospy.logdebug('parse_and_publish: Publishing message: ' + str(msg))
            rospy.loginfo('Publishing description for task \'%s\'', msg.task_name)
            self.pub.publish(msg)
            return msg
        except UnhandledValue as e:
            rospy.logerr('parse_and_publish: Could not parse yaml description: %s: %s', type(e), e)
        except Exception as e:
            rospy.logerr('parse_and_publish: Could not parse yaml description: %s: %s', type(e), e)

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
            f_belt = YamlPars0r.parse_conveyor_belt(YamlPars0r.get_dict_value(y, 'conveyor_belt'))
            rospy.loginfo("<<<<<<Mher>>>>>\r\n %s"%YamlPars0r.get_dict_value(y, 'puzzle_fixture'))
            puzzle_fixture = YamlPars0r.parse_puzzle_fixture(YamlPars0r.get_dict_value(y, 'puzzle_fixture'))
            relative_puzzle_part_target_poses = YamlPars0r.parse_relative_puzzle_part_target_poses(
                YamlPars0r.get_dict_value(y, 'relative_puzzle_part_target_poses')
            )
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
                two_axes_table_speed_limit=YamlPars0r.get_dict_value(YamlPars0r.get_dict_value(y, 'two_axes_table'),
                                                                     'speed_limit', list),
                conveyor_belt=f_belt,
                puzzle_fixture=puzzle_fixture,
                relative_puzzle_part_target_poses = relative_puzzle_part_target_poses
            )
            return msg
        else:
            return Task()
    @staticmethod
    def parse_puzzle_fixture(inp):
        if inp is not None:
            f_pose = YamlPars0r.parse_pose(YamlPars0r.get_dict_value(inp, 'pose'))
            return f_pose

    @staticmethod
    def parse_relative_puzzle_part_target_poses(inp):
        parts = []
        if inp is not None:
            pp = None
            for key in inp:
                pp = PuzzlePart()
                pp.name = key
                pp.pose = YamlPars0r.parse_pose(YamlPars0r.get_dict_value(inp, key))
                parts.append(pp)
        return parts

    @staticmethod
    def parse_conveyor_belt(belt):
        rospy.loginfo('parse_conveyor_belt')
        belt_msg = ConveyorBelt()
        if belt is None:
            return belt_msg
        f_move_direction_and_length = YamlPars0r.get_dict_value(belt, 'move_direction_and_length', list)
        move_direction_and_length_msg = Vector3()
        try:
            move_direction_and_length_msg.x = f_move_direction_and_length[0]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: x value in f_move_direction_and_length is not set.')
        try:
            move_direction_and_length_msg.y = f_move_direction_and_length[1]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: y value in f_move_direction_and_length is not set.')
        try:
            move_direction_and_length_msg.z = f_move_direction_and_length[2]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: z value in f_move_direction_and_length is not set.')
        belt_msg.move_direction_and_length = move_direction_and_length_msg

        f_drop_center_point = YamlPars0r.get_dict_value(belt, 'drop_center_point', list)
        drop_center_point_msg = Vector3()
        try:
            drop_center_point_msg.x = f_drop_center_point[0]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: x value in drop_center_point is not set.')
        try:
            drop_center_point_msg.y = f_drop_center_point[1]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: y value in drop_center_point is not set.')
        try:
            drop_center_point_msg.z = f_drop_center_point[2]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: z value in drop_center_point is not set.')
        belt_msg.drop_center_point = drop_center_point_msg

        f_drop_deviation = YamlPars0r.get_dict_value(belt, 'drop_deviation', list)
        drop_deviation_msg = Vector3()
        try:
            drop_deviation_msg.x = f_drop_deviation[0]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: x value in drop_deviation is not set.')
        try:
            drop_deviation_msg.y = f_drop_deviation[1]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: y value in drop_deviation is not set.')
        try:
            drop_deviation_msg.z = f_drop_deviation[2]
        except IndexError:
            rospy.loginfo('parse_conveyor_belt: z value in drop_deviation is not set.')
        belt_msg.drop_deviation = drop_deviation_msg

        belt_msg.start_speed = YamlPars0r.get_dict_value(belt, 'start_speed', float)
        belt_msg.end_speed = YamlPars0r.get_dict_value(belt, 'end_speed', float)
        belt_msg.n_objects = YamlPars0r.get_dict_value(belt, 'n_objects')
        belt_msg.object_template = YamlPars0r.get_dict_value(belt, 'object_template')
        return belt_msg

    @staticmethod
    def parse_task_type(task):
        rospy.loginfo('parse_task_type')
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
        rospy.loginfo('parse_target_zones')
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
        rospy.loginfo('parse_sensors')
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
        rospy.loginfo('twist_stamped')
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
        rospy.loginfo('parse_twist')
        rospy.logdebug('parse_twist: parsing ' + str(twist))
        f_linear = Vector3(float(0), float(0), float(0))
        f_angular = Vector3(float(0), float(0), float(0))
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
    def parse_pose(pose):
        rospy.loginfo('parse_pose')
        f_pose = Pose()
        f_pose.position.x = pose[0]
        f_pose.position.y = pose[1]
        f_pose.position.z = pose[2]
        f_pose.orientation = Quaternion(*quaternion_from_euler(pose[3], pose[4], pose[5]))
        return f_pose

    @staticmethod
    def parse_objects(objects):
        rospy.loginfo('parse_objects')
        parsed_objects = []
        if objects is None:
            return parsed_objects
        for obj in objects:
            f_name = obj
            rospy.logdebug('parse_objects: objects[obj]: %s', objects[obj])
            f_color = YamlPars0r.get_dict_value(objects[obj], 'color')
            f_description = YamlPars0r.get_dict_value(objects[obj], 'description')
            try:
                f_primitives = YamlPars0r.parse_primitives(YamlPars0r.get_dict_value(objects[obj], 'shape'))
            except UnhandledValue as e:
                rospy.logerr('parse_objects: Exception %s while parsing primitives: %s', e, f_primitives)
                raise e
            f_material = YamlPars0r.get_dict_value(objects[obj], 'surface_material')

            rospy.logdebug('parse_objects: f_color: %s', f_color)
            rospy.logdebug('parse_objects: f_description: %s', f_description)
            rospy.logdebug('parse_objects: f_primitives: %s', f_primitives)
            rospy.logdebug('parse_objects: f_material: %s', f_material)
            obj = Object(primitives=f_primitives[0],
                         primitive_poses=f_primitives[1],
                         primitive_densities=f_primitives[2])
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
    def parse_primitives(primitives):
        rospy.loginfo('parse_primitives')
        rospy.logdebug('parse_shapes: parsing primitives: %s', primitives)
        parsed_primitives = []
        primitive_poses = []
        densities = []
        if primitives is None:
            return parsed_primitives
        for primitive in primitives:
            f_type = YamlPars0r.get_dict_value(primitive, 'type')
            print(f_type)
            if f_type == 'cylinder':
                f_type = SolidPrimitive.CYLINDER
            elif f_type == 'box':
                f_type = SolidPrimitive.BOX
            elif f_type == 'sphere':
                f_type = SolidPrimitive.SPHERE
            else:
                raise UnhandledValue("Unhandled shape type: " + str(f_type))
            f_pose = YamlPars0r.parse_pose(YamlPars0r.get_dict_value(primitive, 'pose'))
            rospy.logdebug('parse_primitives: f_pose: %s', f_pose)
            f_density = YamlPars0r.get_dict_value(primitive, 'density', float)
            f_dimensions = []
            rospy.logdebug('parse_primitives: f_type: %s', f_type)
            if f_type == SolidPrimitive.BOX:
                f_dimensions = [float('inf')] * 3
                f_size = YamlPars0r.get_dict_value(primitive, 'size')
                try:
                    f_dimensions[SolidPrimitive.BOX_X] = f_size[0]
                except IndexError:
                    rospy.loginfo('x value in dimensions is not set.')
                try:
                    f_dimensions[SolidPrimitive.BOX_Y] = f_size[1]
                except IndexError:
                    rospy.loginfo('Y value in dimensions is not set.')
                try:
                    f_dimensions[SolidPrimitive.BOX_Z] = f_size[2]
                except IndexError:
                    rospy.loginfo('z value in dimensions is not set.')
            elif f_type == SolidPrimitive.CYLINDER:
                f_dimensions = [float('inf')] * 2
                f_length = YamlPars0r.get_dict_value(primitive, 'length', float)
                f_radius = YamlPars0r.get_dict_value(primitive, 'radius', float)
                if f_length is not None:
                    f_dimensions[SolidPrimitive.CYLINDER_HEIGHT] = f_length
                if f_radius is not None:
                    f_dimensions[SolidPrimitive.CYLINDER_RADIUS] = f_radius
            elif f_type == SolidPrimitive.SPHERE:
                f_dimensions = [float('inf')] * 1
                f_radius = YamlPars0r.get_dict_value(primitive, 'radius', float)
                if f_radius is not None:
                    f_dimensions[SolidPrimitive.SPHERE_RADIUS] = f_radius
            else:
                raise UnhandledValue("Unhandled primitive type: " + str(f_type))
            primitive = SolidPrimitive(dimensions=f_dimensions,
                                       type=f_type)
            parsed_primitives.append(primitive)
            primitive_poses.append(f_pose)
            densities.append(f_density)
        return [parsed_primitives, primitive_poses, densities]

    def kill(self, sig, frame):
        rospy.loginfo('Caught ctrl-c. Shutting down.')
        self._subscriber.unregister()
        rospy.signal_shutdown("Shutting down YAML pars0r")
        sys.exit()

    def __del__(self):
        print('YAML destruct0r.')
