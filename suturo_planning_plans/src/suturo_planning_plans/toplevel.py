import rospy
import smach
import smach_ros
import task1


def toplevel_plan():
    rospy.init_node('suturo_planning_execution')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Task1', task1.Task1(),
                               transitions={'outcome1': 'outcome5',
                                            'outcome2': 'outcome4'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('toplevel', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

