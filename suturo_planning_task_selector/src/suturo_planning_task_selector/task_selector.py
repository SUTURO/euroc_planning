import rospy
import threading
from euroc_c2_msgs.srv import *
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r
from suturo_planning_interface import utils

task_stopped = False
task_saved = False
yaml_description = None


def start_task(scene):
    print 'Starting ' + scene
    global yaml_description
    rospy.wait_for_service('euroc_c2_task_selector/start_simulator')

    try:
        start_simulator = rospy.ServiceProxy('euroc_c2_task_selector/start_simulator', StartSimulator)
        yaml_description = start_simulator('C2T03#8112895', scene).description_yaml
        rospy.loginfo('Starting YAML pars0r.')
        yaml_parser = YamlPars0r()
        rospy.loginfo('Started YAML pars0r.')
        # signal.signal(signal.SIGINT, lambda sig, frame: yaml_parser.kill(sig, frame))
        return yaml_parser.parse_and_publish(yaml_description)
    except rospy.ServiceException, e:
        print "Service call to start task failed: %s"%e


def stop_task():
    print 'Stopping task'
    if utils.manipulation is not None:
        print "Print Manipulation Shit:"
        utils.manipulation.print_manipulation()
    print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    try:
        print('Waiting for service euroc_c2_task_selector/stop_simulator.')
        rospy.wait_for_service('euroc_c2_task_selector/stop_simulator', timeout=60)
        print('Service euroc_c2_task_selector/stop_simulator ready.')
        stop_simulator = rospy.ServiceProxy('euroc_c2_task_selector/stop_simulator', StopSimulator)
        global task_stopped
        task_stopped = True
        print 'Exiting stop_task()'
        return stop_simulator()
    except rospy.ServiceException, e:
        print "Service call to stop task failed: %s"%e
    except rospy.ROSException, e:
        print('Waiting for service service euroc_c2_task_selector/stop_simulator timed out.')


def save_task():
    global task_saved
    print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    print('Waiting for service /euroc_interface_node/save_log.')
    try:
        rospy.wait_for_service('/euroc_interface_node/save_log', timeout=60)
        rospy.loginfo('Saving log')
        save_log = rospy.ServiceProxy('/euroc_interface_node/save_log', SaveLog)
        save_log()
        task_saved = True
    except rospy.ServiceException, e:
        print "Service call to save log failed: %s"%e
    except rospy.ROSException, e:
        print('Waiting for service /euroc_interface_node/save_log timed out.')