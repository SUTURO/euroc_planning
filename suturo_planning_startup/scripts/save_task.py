#!/usr/bin/env python

import rospy
from euroc_c2_msgs.srv import SaveLog


def save_task():
    print('save_task()')
    rospy.init_node('save_task', log_level=rospy.DEBUG)
    rospy.loginfo('Waiting for service.')
    rospy.wait_for_service('/euroc_interface_node/save_log')
    rospy.loginfo('Saving log')
    try:
        save_log = rospy.ServiceProxy('/euroc_interface_node/save_log', SaveLog)
        return save_log()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    finally:
        rospy.loginfo('Shutting down save_task node.')
        rospy.signal_shutdown('Terminating save_task().')

if __name__ == '__main__':
    save_task()