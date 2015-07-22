#!/usr/bin/env python
import rospy
import suturo_planning_interface
from suturo_head_mover_msgs.srv import Hammertime

def start_hammertime_service():
    rospy.init_node('suturo_hammertime')
    s = rospy.Service('/suturo/hammertime', suturo_head_mover_msgs.Hammertime, hammertime_service_call)
    print 'service is announced'
    rospy.spin()

def hammertime_service_call(req):
    exit_handler()
    return suturo_head_mover_msgs.srv.Hammertime(True)
    
    
    
class HammerTime(object):
    
    def __init__(self):
        start_hammertime_service()
    

    def exit_handler(self):
        print 'exit handler called!'
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
    ht = HammerTime()