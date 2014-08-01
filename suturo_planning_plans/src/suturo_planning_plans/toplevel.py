import rospy
import smach
import smach_ros
import threading
import task1
import start_nodes


def toplevel_plan():
    rospy.init_node('suturo_planning_execution')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with sm:
        smach.StateMachine.add('StartPerception', start_nodes.StartPerception(),
                               transitions={'success': 'StartSimulation',
                                            'fail': 'StartPerception'})
        smach.StateMachine.add('StartSimulation', start_nodes.StartSimulation(),
                               transitions={'success': 'StartManipulation',
                                            'fail': 'StartSimulation'})
        smach.StateMachine.add('StartManipulation', start_nodes.StartManipulation(),
                               transitions={'success': 'ExecuteTask1',
                                            'fail': 'StartManipulation'})
        smach.StateMachine.add('ExecuteTask1', task1.Task1(),
                               transitions={'success': 'StopSimulation',
                                            'fail': 'ExecuteTask1'})
        smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(),
                               transitions={'success': 'success',
                                            'fail': 'fail'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('toplevel', sm, '/SM_ROOT')
    sis.start()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()

