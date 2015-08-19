#!/usr/bin/env python
import rospy
#import suturo_planning_interface
from suturo_head_mover_msgs.srv import Hammertime, HammertimeRequest, HammertimeResponse
#from suturo_planning_interface import start_nodes
from os import system
from os.path import basename
import subprocess
    
    
    
class HammerTime(object):
    
    def __init__(self):
        self.start_hammertime_service()
        
    def start_hammertime_service(self):
        s = rospy.Service('/suturo/hammertime', Hammertime, self.hammertime_service_call)
        print 'service is announced'
        rospy.spin()

    def hammertime_service_call(self, req):
        self.exit_handler()
        return HammertimeResponse(True)

    def exit_handler(self):
        print 'exit handler called!'
        kill_cmd = "kill $(ps aux | grep ros | grep -v grep | awk '{print $2}')"
        kill_cmd = "kill -9 $(ps aux | grep 'ros\\|euroc_c2s1/bin/gzserver\\|euroc_c2s1/bin/task_selector\\|euroc_c2s1/bin/gzclient\\|mongodb_log_' | grep -v grep | awk '{print $2}')"
        task = subprocess.Popen(kill_cmd, shell=True, stdout=subprocess.PIPE)
        data = task.stdout.read()
        assert task.wait() == 0
        print data
        
        #nodes_to_kill = ["/planning", "/yaml_pars0r", "arm_base_controller", "/arm_controller", "/base_controller", "/euroc_c2_task_selector", "/euroc_interface_node", "/gripper_controller", " /image_compression", "/json_prolog", "/manipulation_controller", "/map_base_footprint_publisher", "/map_odom_publisher", "/mongodb_log_worker_0_logged_images_out_compressed", "/mongodb_log_worker_1_logged_designators", "/mongodb_log_worker_2_tf", "/move_group", "/publish_objects_tf_frames", "/robot_state_publisher", "/scan_map_services", "/semrec_ros", "/telemetry_to_joint_state", "/topic_logging"]
        #for kill_node in nodes_to_kill:
            #node_name = basename(kill_node)
            #print "killing node " + kill_node
            #print "node basename: " + node_name
            ##system('ps aux | grep '+node_name+' | grep -v grep | awk \'{print $2}\' > /tmp/hammertime_kill.pid')
            #get_pid_cmd = "ps aux | grep "+node_name+" | grep -v grep | awk \'{print $2}\'"
            #print "cmd: " + get_pid_cmd
            #task = subprocess.Popen(get_pid_cmd, shell=True, stdout=subprocess.PIPE)
            #data = task.stdout.read()
            #assert task.wait() == 0
            #print data
            #pids = filter(None, data.split('\n'))
            #for pid in pids:
                #print "killing pid \"" + pid + "\""
                #try:
                    #pid = int(pid)
                    #pid += 1
                    #pid -= 1
                    #res = system('kill -9 '+str(pid))
                #except TypeError:
                    #print "\""+pid+"\" doesn't seem to be a pid"
            ##res = system('rosnode kill '+kill_node)
            #print "-"*50
        #print "done."
        
        
        #global __handling_exit
        #global _save_log
        #print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
        #if __handling_exit:
            #print('startup: Already handling exit.')
            #return
        #__handling_exit = True
        #print('startup: Checking for task selector.')
        #global _pro_task_selector
        #if _pro_task_selector is not None:
            #print 'Stopping TaskSelector'
            #print 'Stopping gazebo'
            #exterminate(_pro_task_selector.pid, signal.SIGINT)
        ##print('Exiting moveit_commander.')
        ##moveit_commander.os._exit(0)
        #print('startup: Exiting exit_handler')
        
    #def rospy_exit_handler(self):
        #print('startup: rospy_exit_handler')
        #global _save_log
        #if not start_nodes.executed_test_node_check:
            #start_nodes.check_node(initialization_time, logging)
        #print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
        #print('Checking stop task.')
        #if not task_selector.task_stopped:
            #rospy.loginfo('Going to stop task.')
            #stop_task()
        #print('Going to sleep a sec.')
        #time.sleep(2)
        #print('startup: Exiting rospy_exit_handler.')

if __name__ == '__main__':
    print 'starting hammertime service'
    rospy.init_node('suturo_hammertime')
    ht = HammerTime()