import rospy
from euroc_c2_msgs.srv import *
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r


def start_task(scene):
    print 'Starting ' + scene
    rospy.wait_for_service('euroc_c2_task_selector/start_simulator')
    try:
        start_simulator = rospy.ServiceProxy('euroc_c2_task_selector/start_simulator', StartSimulator)
        yaml_description = start_simulator('suturo', scene).description_yaml
        yaml_parser = YamlPars0r()
        return yaml_parser.parse_and_publish(yaml_description)
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
