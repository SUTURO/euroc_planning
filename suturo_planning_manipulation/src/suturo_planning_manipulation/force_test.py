#!/usr/bin/env python
from rospy.rostime import Duration
from suturo_perception_msgs.srv import GetGripper
from manipulation import *
from tabulate import tabulate
import numpy
import copy

__author__ = 'benny'


def get_position(pose):
    l = []
    l.append(pose.primitive_poses[0].position.x)
    l.append(pose.primitive_poses[0].position.y)
    l.append(pose.primitive_poses[0].position.z)
    return l

def pos_calc_test(m):
    # create service
    service = rospy.ServiceProxy("/suturo/GetGripper", GetGripper)
    # get the first perception
    resp = service("firstConveyorCall,centroid,cuboid")
    object1 = resp.objects[0].object
    time1 = resp.stamp
    # wait some time until the second perception
    rospy.sleep(Duration.from_sec(1))
    resp = service("centroid,cuboid")
    object2 = resp.objects[0].object
    time2 = resp.stamp
    # transform the points into odom_combined
    pose1 = m.transform_to(object1)
    pose2 = m.transform_to(object2)
    # calculate the vector from the points of first and second perception
    dir = numpy.array([(pose2.primitive_poses[0].position.x - pose1.primitive_poses[0].position.x), (
    pose2.primitive_poses[0].position.y - pose1.primitive_poses[0].position.y)])
    # dir_dist = sqrt(pow(dir[0], 2) + pow(dir[1], 2))
    # get the duration between the two looks
    time_12 = time2 - time1

    # this stuff is done to test the accuracy of the extrapolation
    # wait some time to get the expected position
    rospy.sleep(Duration.from_sec(5))
    resp = service("centroid,cuboid")
    object3 = resp.objects[0].object
    time3 = resp.stamp
    pose3 = m.transform_to(object3)
    # Get the duration between the first perception and now
    time_13 = time3 - time1
    # get the relation between the first and second duration
    diff_time = float(time_13.to_sec()) / float(time_12.to_sec())
    # table = [["stamps: ", time1, time2, time3], ["duration: ", time_12, time_13, diff_time]]
    # print tabulate(table)
    # calculate the new longer vector (scalar multiplication by hand)
    dir_13 = numpy.array([diff_time * dir[0], diff_time * dir[1]])
    # add this new vector on the points from the first perception
    pose_comp = copy.deepcopy(pose1)
    pose_comp.primitive_poses[0].position.x += dir_13[0]
    pose_comp.primitive_poses[0].position.y += dir_13[1]
    table = [get_position(pose1), get_position(pose2), get_position(pose3), get_position(pose_comp)]
    print tabulate(table)
    # the percentage of deviation is returned
    dev = get_position(pose3)[1] / get_position(pose_comp)[1] * 100
    print dev
    return dev



if __name__ == '__main__':
    rospy.init_node('force_test', anonymous=True)
    m = Manipulation()
    # m.open_gripper()
    # t_point = geometry_msgs.msg.PoseStamped()
    # t_point.header.frame_id = "/odom_combined"
    # t_point.pose.position = geometry_msgs.msg.Point(0, 0.1, 0.5)
    # t_point.pose.orientation = geometry_msgs.msg.Quaternion(0, 0, 1, 0)
    # print m.move_base(t_point)
    # m.close_gripper()
    # m.pan_tilt(0, 0)
    # tfs = TorqueForceService()
    # r = rospy.Rate(10)
    # while not rospy.is_shutdown():
    # print tfs.get_values()
    # r.sleep()
    m.move_to("scan_conveyor_pose1")
    pos_calc_test(m)