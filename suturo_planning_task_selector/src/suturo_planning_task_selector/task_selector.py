import rospy
import subprocess
from euroc_c2_msgs.srv import *
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r

task_stopped = False
task_saved = False


def start_task(scene):
    print 'Starting ' + scene
    rospy.wait_for_service('euroc_c2_task_selector/start_simulator')

    try:
        start_simulator = rospy.ServiceProxy('euroc_c2_task_selector/start_simulator', StartSimulator)
        yaml_description = start_simulator('C2T03#8112895', scene).description_yaml
        yaml_parser = YamlPars0r()
        # signal.signal(signal.SIGINT, lambda sig, frame: yaml_parser.kill(sig, frame))
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
    global task_stopped
    task_stopped = True


def save_task():
    sp = subprocess.Popen('rosrun suturo_planning_startup save_task.py', shell=True)
    sp.wait()
    global task_saved
    task_saved = True
