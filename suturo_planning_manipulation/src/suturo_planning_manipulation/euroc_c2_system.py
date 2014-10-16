# Helper class for the EuRoC ROS interface: Maps all ros services and topics to function calls and variables. 
# Copyright (C) 2014 German Aerospace Center (DLR)

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.'

# Neither the name of the German Aerospace Center (DLR)  nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.

import roslib; roslib.load_manifest('euroc_c2_demos')
import rospy
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from math import pi
import copy

# The euroc_c2_selector class maps all ros services required to start and stop the simulation
class euroc_c2_selector:
    def __init__(self):

        euroc_task_services = [
            ('list_scenes', ListScenes),
            ('start_simulator', StartSimulator),
            ('stop_simulator', StopSimulator)
            ]

        # List of available services and their types
        self.services = dict()
        euroc_task_node = '/euroc_c2_task_selector/'
        for service in euroc_task_services:
            service_name =  euroc_task_node + service[0]
            print 'Waiting for service %s' % service_name
            rospy.wait_for_service(service_name)
            self.services[service[0]] = rospy.ServiceProxy(service_name, service[1])
            print 'Found service %s' % service_name

    def list_scenes(self):
        list_scenes_service = self.services['list_scenes']
        resp = list_scenes_service()
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.scenes

    def start_simulator(self, user_id, scene):
        start_simulator_service = self.services['start_simulator']
        resp = start_simulator_service(user_id, scene)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.description_yaml

    def stop_simulator(self):
        stop_simulator_service = self.services['stop_simulator']
        resp = stop_simulator_service()
        if resp.error_message:
            raise Exception(resp.error_message)
        
    def reset(self):
        self.stop_simulator()

# The euroc_c2_system class maps all ros services and topics to functions and variables,
# which are required to interact with the simulation
class euroc_c2_system:
    def __init__(self):

        rospy.init_node('euroc_c2_system_node', anonymous=True)
        
        # List of available services and their types
        euroc_c2_services = [
            ('move_along_joint_path', MoveAlongJointPath),
            ('get_timing_along_joint_path', GetTimingAlongJointPath),
            ('set_stop_conditions', SetStopConditions),
            ('get_estimated_external_force', GetEstimatedExternalForce),
            ('get_forward_kinematics', GetForwardKinematics),
            ('search_ik_solution', SearchIkSolution),
            ('enable_servo_mode', EnableServoMode),
             ('set_servo_target', SetServoTarget),
            ('request_next_object', RequestNextObject),
            ('set_sensor_update_divisor', SetSensorUpdateDivisor),
            ('set_object_load', SetObjectLoad),
            ('save_log', SaveLog)
        ]

        euroc_interface_node = '/euroc_interface_node/'
        self.services = dict()
        for service in euroc_c2_services:
            service_name =  euroc_interface_node + service[0]
            rospy.wait_for_service(service_name)
            self.services[service[0]] = rospy.ServiceProxy(service_name, service[1])

        self.telemetry = Telemetry()
        self.lwr_fk = PoseStamped()
        self.scene_rgb_image = Image()
        self.scene_depth_image = Image()
        self.tcp_depth_image = Image()
        self.tcp_rgb_image = Image()

        rospy.Subscriber(euroc_interface_node + 'telemetry', Telemetry, self.on_telemetry);
        rospy.Subscriber(euroc_interface_node + 'lwr_base_to_tcp', PoseStamped, self.on_lwr_fk);
        rospy.Subscriber(euroc_interface_node + 'cameras/tcp_rgb_cam', Image, self.on_tcp_rgb_image);
        rospy.Subscriber(euroc_interface_node + 'cameras/tcp_depth_cam', Image, self.on_tcp_depth_image);
        rospy.Subscriber(euroc_interface_node + 'cameras/scene_rgb_cam', Image, self.on_scene_rgb_image);
        rospy.Subscriber(euroc_interface_node + 'cameras/scene_depth_cam', Image, self.on_scene_depth_image);

        self.gripper_joints = ['gripper']
        self.axis_joints = ['axis_x', 'axis_y']
        self.lwr_joints = ['lwr_joint_' + str(i+1) for i in range(7)]
        self.cam_joints = ['cam_pan', 'cam_tilt']
        self.all_joints = self.axis_joints + self.lwr_joints + self.gripper_joints + self.cam_joints
        self.cameras = ['tcp_rgb_cam', 'tcp_depth_cam', 'scene_rgb_cam', 'scene_depth_cam']

        self.reset()

    def reset(self):
        self.set_stop_conditions([])
        for camera in self.cameras:
            self.set_sensor_update_divisor(camera, 1)
        path = [Configuration(q=[0 for i in range(len(self.all_joints))])]
        self.enable_servo_mode(False);
        self.set_object_load(0.0, Vector3(0.0, 0.0, 0.0))
        self.move(path, self.all_joints)

    def on_telemetry(self, telemetry):
        self.telemetry = telemetry
    
    def on_lwr_fk(self, fk):
        self.lwr_fk = fk

    def on_scene_rgb_image(self, image):
        self.scene_rgb_image = image

    def on_scene_depth_image(self, image):
        self.scene_depth_image = image

    def on_tcp_rgb_image(self, image):
        self.tcp_rgb_image = image

    def on_tcp_depth_image(self, image):
        self.tcp_depth_image = image

    def get_telemetry(self):
        return self.telemetry

    def get_lwr_fk(self):
        return self.lwr_fk;

    def get_tcp_rgb(self):
        return self.tcp_rgb_image

    def get_tcp_depth(self):
        return self.tcp_depth_image

    def get_scene_rgb(self):
        return self.scene_rgb_image

    def get_scene_depth(self):
        return self.scene_depth_image

    def move(self, 
             path, 
             joint_names, 
             lwr_vel=90, 
             lwr_acc=500, 
             tvel=0, 
             tacc=0,
             rvel=0, 
             racc=0,
             axis_vel=0.5, 
             axis_acc=1, 
             gripper_vel=0.05, 
             gripper_acc=0.1, 
             lwr_path_unit="deg", 
             start_sim_time=0
             ):
        move_service = self.services['move_along_joint_path']
        joint_limits=[];
        cartesian_limits = CartesianLimits();
        cartesian_limits.translational.max_velocity = tvel
        cartesian_limits.translational.max_acceleration = tacc
        cartesian_limits.rotational.max_velocity = rvel
        cartesian_limits.rotational.max_acceleration = racc

        rad_path = copy.deepcopy(path)

        for i in range(len(joint_names)):
            limit = Limits();
            joint_name = joint_names[i]
            if joint_name[0:4] == "axis":
                limit.max_velocity = axis_vel
                limit.max_acceleration = axis_acc
            if joint_name[0:3] == "lwr" or joint_name[0:3] == "cam":
                limit.max_velocity = lwr_vel*pi/180.0
                limit.max_acceleration = lwr_acc*pi/180.0
                if lwr_path_unit == "deg":
                    for j in range(len(path)):
                        rad_path[j].q[i] = path[j].q[i]*pi/180.0
            if joint_name[0:3] == "cam":
                limit.max_velocity = 0.4
                limit.max_acceleration = 2.0
            if joint_name == "gripper":
                limit.max_velocity = gripper_vel
                limit.max_acceleration = gripper_acc
            joint_limits.append(limit)

        ros_start_time = rospy.Time()
        ros_start_time.from_seconds(start_sim_time)

        resp = move_service(joint_names, rad_path, ros_start_time, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.stop_reason


    def get_timing(self, 
                   path, 
                   joint_names, 
                   start_pose,
                   lwr_vel=90, 
                   lwr_acc=500, 
                   tvel=0, 
                   tacc=0,
                   rvel=0, 
                   racc=0, 
                   axis_vel=0.5, 
                   axis_acc=1, 
                   gripper_vel=0.05, 
                   gripper_acc=0.1, 
                   lwr_path_unit="deg"
                   ):
        get_timing_service = self.services['get_timing_along_joint_path']
        joint_limits=[];
        cartesian_limits = CartesianLimits();
        cartesian_limits.translational.max_velocity = tvel
        cartesian_limits.translational.max_acceleration = tacc
        cartesian_limits.rotational.max_velocity = rvel
        cartesian_limits.rotational.max_acceleration = racc

        rad_path = copy.deepcopy(path)

        for i in range(len(joint_names)):
            limit = Limits();
            joint_name = joint_names[i]
            if joint_name[0:4] == "axis":
                limit.max_velocity = axis_vel
                limit.max_acceleration = axis_acc
            if joint_name[0:3] == "lwr" or joint_name[0:3] == "cam":
                limit.max_velocity = lwr_vel*pi/180.0
                limit.max_acceleration = lwr_acc*pi/180.0
                if lwr_path_unit == "deg":
                    for j in range(len(path)):
                        rad_path[j].q[i] = path[j].q[i]*pi/180.0
            if joint_name[0:3] == "cam":
                limit.max_velocity = 0.4
                limit.max_acceleration = 2.0
            if joint_name == "gripper":
                limit.max_velocity = gripper_vel
                limit.max_acceleration = gripper_acc
            joint_limits.append(limit)

        resp = get_timing_service(joint_names, start_pose, rad_path, joint_limits, cartesian_limits)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.time_at_via_point

    def set_stop_conditions(self, stop_conditions):
        set_stop_condition_service = self.services['set_stop_conditions']
        resp = set_stop_condition_service(stop_conditions)
        if resp.error_message:
            raise Exception(resp.error_message)
    
    def get_estimated_external_force(self):
        get_estimated_external_force_service = self.services['get_estimated_external_force']
        resp = get_estimated_external_force_service()
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.external_force

    def get_forward_kinematics(self, configuration):
        get_forward_kinematics_service = self.services['get_forward_kinematics']
        resp = get_forward_kinematics_service(configuration)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.ee_frame

    def request_next_object(self):
        request_next_object_service = self.services['request_next_object']
        resp = request_next_object_service()
        if resp.error_message:
            raise Exception(resp.error_message)

    def search_ik_solution(self, start, tcp_frame):
        search_ik_solution_service = self.services['search_ik_solution']
        resp = search_ik_solution_service(start, tcp_frame)
        if resp.error_message:
            raise Exception(resp.error_message)
        return resp.solution

    def set_sensor_update_divisor(self, sensor_name, update_divisor):
        set_sensor_update_divisor_service = self.services['set_sensor_update_divisor']
        resp = set_sensor_update_divisor_service(sensor_name, update_divisor)
        if resp.error_message:
            raise Exception(resp.error_message)

    def enable_servo_mode(self, servo_mode_active):
        enable_servo_mode_service = self.services['enable_servo_mode']
        resp = enable_servo_mode_service(servo_mode_active)
        if resp.error_message:
            raise Exception(resp.error_message)
    
    def set_servo_target(self, joint_names, target):
        set_servo_target_service = self.services['set_servo_target']
        resp = set_servo_target_service(joint_names, target)
        if resp.error_message:
            raise Exception(resp.error_message)

    def set_object_load(self, mass, center_of_mass):
        set_object_load_service = self.services['set_object_load']
        resp = set_object_load_service(mass, center_of_mass)
        if resp.error_message:
            raise Exception(resp.error_message)

    def save_log(self):
        save_log_service = self.services['save_log']
        resp = save_log_service()
        if resp.error_message:
            raise Exception(resp.error_message)

