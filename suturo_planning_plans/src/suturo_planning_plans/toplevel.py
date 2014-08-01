import rospy
import smach
import smach_ros
import threading
import task1
import start_nodes


def toplevel_plan():
    rospy.init_node('suturo_planning_execution')

    # Create a SMACH state machine
    toplevel = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with toplevel:

        execute_task1 = smach.StateMachine(outcomes=['success', 'fail'])

        with execute_task1:
            smach.StateMachine.add('InitSimulation', InitSimulation('task1_v1'),
                                   transitions={'success': 'Task1Plan',
                                                'fail': 'fail'})
            smach.StateMachine.add('Task1Plan', task1.Task1(),
                                   transitions={'success': 'StopSimulation',
                                                'fail': 'Task1Plan'})
            smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(),
                                   transitions={'success': 'success',
                                                'fail': 'fail'})

        smach.StateMachine.add('ExecuteTask1', execute_task1,
                               transitions={'success': 'success',
                                            'fail': 'fail'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('toplevel', toplevel, '/SM_ROOT')
    sis.start()

    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=toplevel.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    toplevel.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()


class InitSimulation(smach.StateMachine):
    def __init__(self, task_name):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=[],
                                    output_keys=[])
        with self:
            smach.StateMachine.add('StartPerception', start_nodes.StartPerception(),
                                   transitions={'success': 'StartSimulation',
                                                'fail': 'StartPerception'})
            smach.StateMachine.add('StartSimulation', start_nodes.StartSimulation(task_name),
                                   transitions={'success': 'StartManipulation',
                                                'fail': 'StartSimulation'})
            smach.StateMachine.add('StartManipulation', start_nodes.StartManipulation(),
                                   transitions={'success': 'success',
                                                'fail': 'StartManipulation'})

