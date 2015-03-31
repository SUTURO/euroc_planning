#!/usr/bin/env python
from rospy.rostime import Duration
from suturo_perception_msgs.srv import GetCameraPerception
from manipulation import *
from tabulate import tabulate
from mathemagie import euler_to_quaternion
from mathemagie import rotate_quaternion
import numpy
import copy

__author__ = 'benny'


def get_position(pose):
    l = []
    l.append(pose.primitive_poses[0].position.x)
    l.append(pose.primitive_poses[0].position.y)
    l.append(pose.primitive_poses[0].position.z)
    return l


def test_extrapolation(dir, m, pose1, pose2, service, time1, time_12):
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
    pose_comp.id = "red_cube"
    print pose_comp
    # print out the table for the different poses for debug
    table = [get_position(pose1), get_position(pose2), get_position(pose3), get_position(pose_comp)]
    print tabulate(table)
    dev = get_position(pose3)[1] / get_position(pose_comp)[1] * 100
    print dev
    return pose_comp


def pos_calc_test(m):
    # create service
    service = rospy.ServiceProxy("/suturo/GetGripper", GetCameraPerception)
    # get the first perception
    resp = service("firstConveyorCall,centroid,cuboid")
    object1 = resp.objects[0].object
    time1 = resp.stamp
    # wait some time until the second perception
    rospy.sleep(Duration.from_sec(0.5))
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

    # set the time in 5 secs
    t_5 = rospy.Time.now() + rospy.Duration(8)
    time_13 = t_5 - time1
    diff_time = float(time_13.to_sec()) / float(time_12.to_sec())
    dir_13 = numpy.array([diff_time * dir[0], diff_time * dir[1]])
    # add this new vector on the points from the first perception
    pose_comp = copy.deepcopy(pose1)
    pose_comp.primitive_poses[0].position.x += dir_13[0]
    pose_comp.primitive_poses[0].position.y += dir_13[1]
    pose_comp.id = "red_cube"
    # print out the table for the different poses for debug
    table = [get_position(pose1), get_position(pose2), get_position(pose_comp)]
    print tabulate(table)
    # return test_extrapolation(dir, m, pose1, pose2, service, time1, time_12)
    return pose_comp

def plan_to(pose, m):
    service = rospy.ServiceProxy("/euroc_interface_node/search_ik_solution", SearchIkSolution)
    config = euroc_c2_msgs.msg.Configuration()
    list = m.get_current_lwr_joint_state()
    for i in range(len(list)):
                config.q.append(list[i])
    print "time before planning"
    print rospy.Time.now().to_sec()
    resp = service(config, pose)
    if resp.error_message:
        raise Exception(resp.error_message)
    print "time after planning"
    print rospy.Time.now().to_sec()
    return resp.solution


if __name__ == '__main__':
    rospy.init_node('force_test', anonymous=True)
    m = Manipulation()
    m.open_gripper()
    m.move_to("scan_conveyor_pose1")
    print "time before calculation"
    print rospy.Time.now().to_sec()
    obj = pos_calc_test(m)
    print "time after calculation"
    print rospy.Time.now().to_sec()
    t_point = geometry_msgs.msg.Pose()
    t_point.position = obj.primitive_poses[0].position
    t_point.position.z += 0.3
    quat = Quaternion()
    quat.x = obj.primitive_poses[0].orientation.x
    quat.y = obj.primitive_poses[0].orientation.y
    quat.z = obj.primitive_poses[0].orientation.z
    quat.w = obj.primitive_poses[0].orientation.w
    t_point.orientation = rotate_quaternion(quat, -pi/2, 0, 0)
    # t_point.orientation = quat
    m.direct_move(plan_to(t_point, m))
    t_point.position.z -= 0.07
    m.direct_move(plan_to(t_point, m))
    m.close_gripper()
    t_point.position.z += 0.1
    m.direct_move(plan_to(t_point, m))
