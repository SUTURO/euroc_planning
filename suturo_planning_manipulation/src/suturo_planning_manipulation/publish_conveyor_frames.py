#!/usr/bin/env python
__author__ = 'thocar'

import rospy
from suturo_msgs.msg import Task
import tf

def cb(msg):
    drop_point = msg.conveyor_belt.drop_center_point
    mdl = msg.conveyor_belt.move_direction_and_length
    target_zone = msg.target_zones[0].target_position

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    print "begin to publish /drop_point, /mdl_middle and /target_zone"
    while not rospy.is_shutdown():
        br.sendTransform((drop_point.x, drop_point.y, drop_point.z),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/drop_point",
                         "/odom_combined")
        rate.sleep()
        br.sendTransform((mdl.x/2, mdl.y/2, mdl.z/2),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/mdl_middle",
                         "/drop_point")
        rate.sleep()
        br.sendTransform((target_zone.x, target_zone.y, target_zone.z),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/target_zone",
                         "/odom_combined")
        rate.sleep()
    pass

def add_conveyor_frames():
    rospy.init_node('/suturo/conveyor_frames_publisher', anonymous=True)

    rospy.Subscriber("suturo/yaml_pars0r", Task, cb)

    rospy.spin()
    print "Oooppss"

if __name__ == '__main__':
    add_conveyor_frames()