import rospy
from euroc_c2_msgs.srv import *


def start_task(scene):
    print 'Starting ' + scene
    rospy.wait_for_service('euroc_c2_task_selector/start_simulator')
    try:
        start_simulator = rospy.ServiceProxy('euroc_c2_task_selector/start_simulator', StartSimulator)
        return start_simulator('suturo', scene)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def stop_task():
    print 'Stopping task'
    rospy.wait_for_service('euroc_c2_task_selector/stop_simulator')
    try:
        stop_simulator = rospy.ServiceProxy('euroc_c2_task_selector/stop_simulator', StopSimulator)
        return stop_simulator()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def save_task():
    print 'Saving log'
    rospy.wait_for_service('/euroc_interface_node/save_log')
    try:
        save_log = rospy.ServiceProxy('/euroc_interface_node/save_log', SaveLog)
        return save_log()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
