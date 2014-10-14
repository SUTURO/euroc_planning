#!/usr/bin/env python

# Python demonstration program for the EuRoC C2S1: Inspects all target zones of task 1 with the tcp camera. 
# Copyright (C) 2014  German Aerospace Center (DLR)

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

from geometry_msgs.msg import * 

from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *

from euroc_c2_system import euroc_c2_selector
from euroc_c2_system import euroc_c2_system

import cv2
from cv_bridge import CvBridge, CvBridgeError

import yaml

def euroc_c2_demo():
    # First we load the euroc_c2_selector class supplied with this tutorial.
    # The class allows for easier access to the euroc c2 task selection
    # ros interface by registering to all available task selection services.
    task_selector = euroc_c2_selector()

    # The list scenes service returns all available scenes / tasks in the EuRoC C2 Simulation Challenge.
    # Each scene has a name and a description specified in yaml format.
    scenes = task_selector.list_scenes();

    # Let's print the names of the recieved scenes
    print 'Found the following scenes for the EuRoC C2 Simulation:'
    for scene in scenes:
        print ' - ' + scene.name

    # The start simulator service starts the simulation with a specified scene 
    # and an user id. We'll select the task 1 scene for now.
    # The start simulator service returns a description of the selected task in yaml format
    task_yaml_string = task_selector.start_simulator('demo user', 'task1_v1')

    # Let's parse and print the description text of the task
    task_yaml = yaml.safe_load(task_yaml_string)
    task_description = task_yaml['description']
    target_zones = task_yaml['target_zones']

    # Let's parse the positions of the target zones and fill them into a list of desired poses
    poses=[]
    for target_zone in sorted(target_zones):
        target_position = target_zones[target_zone]['target_position']
        position = Point()
        position.x = target_position[0]
        position.y = target_position[1]
        # 0.5 meters above the target zone
        position.z = 0.5
        # orient the tcp towards the table
        orient_towards_table = Quaternion(1.0, 0.0, 0.0, 0.0)
        poses.append( Pose(position, orient_towards_table) )
        
    # Now we load the euroc_c2_system class supplied with this tutorial which regsters with all
    # services of the euroc system interface after the simulation is started.
    system = euroc_c2_system()

    nr_lwr_joints = len(system.lwr_joints)
    current_configuration = Configuration(q = [0.0] * nr_lwr_joints)

    # Construct an open cv image to display the tcp camera image
    bridge = CvBridge()
    cv_window_name = 'LWR tcp camera'
    cv2.namedWindow(cv_window_name, 1)
    cv2.moveWindow(cv_window_name, 0, 0)

    pose_index = 0;
    while not rospy.is_shutdown():

        # Get the current configuration from the telemetry message
        current_telemetry = system.get_telemetry()
        for (i, joint) in enumerate(system.lwr_joints):
            telemetry_index = current_telemetry.joint_names.index(joint)
            current_configuration.q[i] = current_telemetry.measured.position[telemetry_index]
        
        # Select the next desired position of the tcp from the target zone poses 
        desired_pose = poses[pose_index];
        
        # Search for the inverse kinematic solution to this pose which lies close to the current configuration
        solution_configuration = system.search_ik_solution(current_configuration, desired_pose)
        print solution_configuration
        
        # Move towards the solution configuration
        path = [solution_configuration]
        system.move(path, system.lwr_joints, lwr_path_unit='rad')
        
        # Display the tcp camera image
        try:
            cv_image = bridge.imgmsg_to_cv2(system.tcp_rgb_image, "bgr8")
        except CvBridgeError, e:
            print
        cv2.imshow(cv_window_name, cv_image)
        cv2.waitKey(250)

        # Update towards the next desired pose
        pose_index = (pose_index + 1) % len(poses)
        
    # When shutting down the simulator early you need to save the log manually with the save_log service
    system.save_log()
    # The stop simulator callback ends the current simulation
    selector.stop_simulator()

if __name__ == '__main__':
    euroc_c2_demo()
