import sys
import rospy
import smach
import smach_ros
import threading
import time
import task1
import task4
import task6
import start_nodes
from suturo_msgs.msg import Task


def handle_uncaught_exception(e):
    print('Uncaught exception: ' + str(e))
    print('Terminating task.')
    rospy.signal_shutdown('Terminating Task due to unhandled exception.')


def toplevel_plan(init_sim, task_list, savelog, initialization_time):

    # Create a SMACH state machine
    toplevel = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with toplevel:
        for task_name in task_list:
            rospy.logdebug('Adding task: %s', task_name)
            smach.StateMachine.add('Execute%s' % task_name,
                                   EurocTask(init_sim, task_name, savelog, initialization_time),
                                   transitions={'success': 'success',
                                                'fail': 'fail'})

    # Create and start the introspection server
    rospy.loginfo('Creating and starting the introspection server.')
    sis = smach_ros.IntrospectionServer('toplevel', toplevel, '/SM_ROOT')
    sis.start()

    # Create a thread to execute the smach container
    rospy.loginfo('Creating a thread to execute the smach container.')

    def execute_task():
        try:
            toplevel.execute()
        except:
            handle_uncaught_exception(sys.exc_info()[0])

    smach_thread = threading.Thread(target=execute_task)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    rospy.loginfo('Request the container to preempt.')
    toplevel.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    rospy.loginfo('Block until everything is preempted.')
    smach_thread.join()


class EurocTask(smach.StateMachine):
    def __init__(self, init_sim, task_name, savelog, initialization_time):
        self.savelog = savelog
        smach.StateMachine.__init__(self, input_keys=[], outcomes=['success', 'fail'])
        self.userdata.initialization_time = initialization_time
        # Associate the task name with a state machine
        plans = {'task1': task1.Task1,
                 'task2': task1.Task1,
                 'task3': task1.Task1,
                 'task4': task4.Task4,
                 'task5': task4.Task4,
                 'task6': task6.Task6}

        # Associate the task name with the parameter for the state machine
        plan_args = {'task1': [False, 'task1'],
                     'task2': [False, 'task2'],
                     'task3': [True, 'task3'],
                     'task4': ['task4'],
                     'task5': ['task5'],
                     'task6': ['task6']}

        # Parses the task name
        task = task_name.split('_')[0]

        with self:
            task_success = 'success'

            if init_sim:
                smach.StateMachine.add('InitSimulation', InitSimulation(task_name),
                                       transitions={'success': task_name,
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(self.savelog),
                                       transitions={'success': 'StopNodes',
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopNodes', start_nodes.StopNodes(),
                                       transitions={'success': 'success',
                                                    'fail': 'fail'})

                task_success = 'StopSimulation'
            else:
                smach.StateMachine.add('GetYaml', GetYaml(),
                                       transitions={'success': task_name,
                                                    'fail': 'fail'})

            smach.StateMachine.add(task_name, plans[task](*plan_args[task]),
                                   transitions={'success': task_success,
                                                'fail': 'fail'})


class InitSimulation(smach.StateMachine):
    def __init__(self, task_name):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['initialization_time'],
                                    output_keys=['objects_found', 'yaml', 'perception_process', 'manipulation_process',
                                                 'classifier_process', 'perception_logger_process',
                                                 'manipulation_logger_process', 'classifier_logger_process'])
        with self:
            # if task_name.split('_')[0] == 'task6':
            #    rospy.loginfo("Task 6 Perception started")
            #    smach.StateMachine.add('StartPerceptionTask6', start_nodes.StartPerceptionTask6(),
            #                           transitions={'success': 'StartSimulation',
            #                                        'fail': 'StartPerception'})
            # else:
            # rospy.loginfo("Normal Perception started")
            smach.StateMachine.add('StartSimulation', start_nodes.StartSimulation(task_name),
                                   transitions={'success': 'StartPerception',
                                                'fail': 'StartSimulation'})
            smach.StateMachine.add('StartPerception', start_nodes.StartPerception(),
                                   transitions={'success': 'StartManipulation',
                                                'fail': 'StartPerception'})
            smach.StateMachine.add('StartManipulation', start_nodes.StartManipulation(),
                                   transitions={'success': 'StartClassifier',
                                                'fail': 'StartManipulation'})
            smach.StateMachine.add('StartClassifier', start_nodes.StartClassifier(),
                                   transitions={'success': 'success',
                                                'fail': 'StartClassifier'})


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

        rospy.loginfo('Got yaml %s'%str(self._yaml))
        userdata.yaml = self._yaml
        self._lock.release()

        return 'success'

    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: %s'%str(self._yaml))
        self._lock.release()

