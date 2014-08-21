import rospy
import smach
import smach_ros
import threading
import time
import task1
import start_nodes
from suturo_msgs.msg import Task
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r


def toplevel_plan(init_sim):

    # Create a SMACH state machine
    toplevel = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with toplevel:

        execute_task1 = smach.StateMachine(outcomes=['success', 'fail'])

        with execute_task1:
            task1_success = 'success'

            if init_sim:
                smach.StateMachine.add('InitSimulation', InitSimulation('task1_v1'),
                                       transitions={'success': 'Task1Plan',
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(),
                                       transitions={'success': 'StopNodes',
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopNodes', start_nodes.StopNodes(),
                                       transitions={'success': 'success',
                                                    'fail': 'fail'})

                task1_success = 'StopSimulation'
            else:
                smach.StateMachine.add('GetYaml', GetYaml(),
                                       transitions={'success': 'Task1Plan',
                                                    'fail': 'fail'})

            smach.StateMachine.add('Task1Plan', task1.Task1(),
                                   transitions={'success': task1_success,
                                                'fail': 'Task1Plan'})


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
                                    output_keys=['objects_found', 'yaml', 'perception_process', 'manipulation_process'])
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


class GetYaml(smach.State):
    _yaml = None

    _lock = None

    def __init__(self):
        self._lock = threading.Lock()
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=[],
                             output_keys=['yaml'])

    def execute(self, userdata):
        subscriber = rospy.Subscriber("suturo/yaml_pars0r", Task, self.parse_yaml)
        rospy.loginfo('Waiting for yaml')

        self._lock.acquire()
        while self._yaml is None:
            self._lock.release()
            time.sleep(0.05)
            self._lock.acquire()

        rospy.loginfo('Got yaml ' + str(self._yaml))
        userdata.yaml = self._yaml
        self._lock.release()

        return 'success'

    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: ' + str(self._yaml))
        self._lock.release()

