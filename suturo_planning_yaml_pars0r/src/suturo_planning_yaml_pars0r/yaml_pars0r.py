#!/usr/bin/env python
import yaml, rospy, threading
from suturo_msgs.msg import task
from std_msgs.msg import String


class Yaml_pars0r:

    def __init__(self):
        self._yaml = None
        self._message = None
        self._lockYaml = threading.Lock()
        self._lockMessage = threading.Lock()
        self.pub = rospy.Publisher('yaml_pars0r', task, queue_size=10, latch=True)
        rospy.init_node('yaml_pars0r_node', anonymous=True)
        self._subscriber = rospy.Subscriber("yaml_pars0r_input", String, self.get_input)

    def get_input(self, msg):
        self._lockMessage.acquire()
        self._yaml = msg.data
        self._lockMessage.release()
        self.publish()

    def get_dict_value(self, dictionary, key):
        if (type(dictionary) is dict) and (key in dictionary):
            val = dictionary[key]
        else:
            val = ""
        print("val: " + val)
        return val

    def publish(self):
        self.parse_yaml()
        self._lockMessage.acquire()
        #rospy.loginfo(message)
        if self._message:
            self.pub.publish(self._message)
        self._lockMessage.release()

    def parse_yaml(self):
        self._lockYaml.acquire()
        if self._yaml:
            y = yaml.load(self._yaml)
        else:
            y = None
        self._lockYaml.release()
        if y:
            msg = task(
                description=self.get_dict_value(y, 'description'),
                log_filename=self.get_dict_value(y, 'log_filename'))
            self._lockYaml.acquire()
            self._message = msg
            self._lockYaml.release()

    def __del__(self):
        rospy.loginfo("Deleting yaml pars0r instance.")
        self._subscriber.unregister()
        rospy.signal_shutdown()

# Testing:
# rostopic pub -1 /yaml_pars0r_input std_msgs/String '{data: "description: there are 3 objects lying on the table (red_cube, green_cylinder, blue_handle).\n  their projected origin needs to be placed in their respective target zones (zone_A,\n  zone_B, zone_C) on the table.\nlog_filename: /home/andz/euroc_c2s1_logs/euroc_c2_s1_20140807_151009_suturo.log\nmast_of_cam:\n  base_pose: [0.92, 0.92, 0, 0, 0, -2.356]\n  pan_tilt_base: [0, 0, 1.1, 0, 0, 0]\n  speed_limit: [1.7453, 1.7453]\nobjects:\n  blue_handle:\n    color: 0000ff\n    description: a blue compound of a cylinder with two cubes\n    shape:\n    - density: 2710\n      length: 0.3\n      pose: [0, 0, 0.175, 0, 0, 0]\n      radius: 0.01\n      type: cylinder\n    - density: 2710\n      pose: [0, 0, 0, 0, 0, 0]\n      size: [0.05, 0.05, 0.05]\n      type: box\n    - density: 2710\n      pose: [0, 0, 0.35, 0, 0, 0]\n      size: [0.05, 0.05, 0.05]\n      type: box\n    surface_material: aluminium\n  green_cylinder:\n    color: 00ff00\n    description: a green cylinder\n    shape:\n    - density: 19302\n      length: 0.1\n      pose: [0, 0, 0, 0, 0, 0]\n      radius: 0.02\n      type: cylinder\n    surface_material: aluminium\n  red_cube:\n    color: ff0000\n    description: a red cube\n    shape:\n    - density: 7850\n      pose: [0, 0, 0, 0, 0, 0]\n      size: [0.05, 0.05, 0.05]\n      type: box\n    surface_material: aluminium\nrobot:\n  gripper_pose: [0, 0, 0.08, 3.1415927, 0, 0]\n  gripper_speed_limit: 0.5\n  gripper_tcp: [0, 0, -0.093, -3.1415927, 0, 0]\n  pose: [0, 0, 0.005, 0, 0, 0]\n  speed_limit: [1.7453, 1.7453, 2.7925, 2.7925, 4.7124, 3.6652, 3.6652]\nsensors:\n  scene_depth_cam:\n    camera:\n      horizontal_fov: 1.047\n      image: {height: 480, width: 640}\n    relative_pose:\n      from: scene_rgb_cam\n      pose: [0.0, -0.04, 0.0, 0, 0, 0]\n    update_rate: 30\n  scene_rgb_cam:\n    camera:\n      horizontal_fov: 1.047\n      image: {height: 480, width: 640}\n    pose: [0.2, 0.02, 0, 0, 0, 0]\n    update_rate: 30\n  tcp_depth_cam:\n    camera:\n      horizontal_fov: 1.047\n      image: {height: 480, width: 640}\n    relative_pose:\n      from: tcp_rgb_cam\n      pose: [0.0, -0.04, 0.0, 0, 0, 0]\n    update_rate: 30\n  tcp_rgb_cam:\n    camera:\n      horizontal_fov: 1.047\n      image: {height: 480, width: 640}\n    pose: [-0.02, 0.0565, -0.063, -1.5708, 1.5708, 0.0]\n    update_rate: 30\ntarget_zones:\n  zone_A:\n    expected_object: red_cube\n    max_distance: 0.05\n    target_position: [0.5, 0.5, 0]\n  zone_B:\n    expected_object: blue_handle\n    max_distance: 0.05\n    target_position: [0.5, -0.5, 0]\n  zone_C:\n    expected_object: green_cylinder\n    max_distance: 0.05\n    target_position: [0.5, 0, 0]\ntask_name: task 1\ntime_limit: 600\ntwo_axes_table:\n  speed_limit: [0.5, 0.5]\n"}'