def rospy_exit_handler():
    print('startup: rospy_exit_handler')
    global _save_log
    if not start_nodes.executed_test_node_check:
        start_nodes.check_node(initialization_time, logging)
    print 'save_log = ' + str(_save_log) + ', rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    print('Checking save task.')
    if _save_log and not task_selector.task_saved:
        rospy.loginfo('Going to save log')
        save_task()
    print('Checking stop task.')
    if not task_selector.task_stopped:
        rospy.loginfo('Going to stop task.')
        stop_task()
    print('Going to sleep a sec.')
    time.sleep(2)
    print('startup: Exiting rospy_exit_handler.')
    
def exit_handler(signum=None, frame=None):
    print('startup: exit_handler')
    print('startup: signum: ' + str(signum) + ', frame: ' + str(frame))
    global __handling_exit
    global _save_log
    print 'rospy.is_shutdown() = ' + str(rospy.is_shutdown())
    if __handling_exit:
        print('startup: Already handling exit.')
        return
    __handling_exit = True
    print('startup: Checking for task selector.')
    global _pro_task_selector
    if _pro_task_selector is not None:
        print 'Stopping TaskSelector'
        print 'Stopping gazebo'
        exterminate(_pro_task_selector.pid, signal.SIGINT)
    #print('Exiting moveit_commander.')
    #moveit_commander.os._exit(0)
    print('startup: Exiting exit_handler')