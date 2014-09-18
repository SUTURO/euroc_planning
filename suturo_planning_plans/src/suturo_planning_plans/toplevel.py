import rospy
import smach
import smach_ros
import threading
import time
import task1
import task3
import start_nodes
from suturo_msgs.msg import Task
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r


def toplevel_plan(init_sim, task_list):

    # Create a SMACH state machine
    toplevel = smach.StateMachine(outcomes=['success', 'fail'])

    # Open the container
    with toplevel:
        for task_name in task_list:
            rospy.logdebug('Adding task: %s', task_name)
            smach.StateMachine.add('Execute%s'%task_name, EurocTask(init_sim, task_name),
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


class EurocTask(smach.StateMachine):
    def __init__(self, init_sim, task_name):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'])

        plans = {'task1': task1.Task1,
                 'task2': task1.Task1,
                 'task3': task1.Task1}

        plan_args = {'task1': [False, 'task1'],
                     'task2': [False, 'task2'],
                     'task3': [True, 'task3']}

        task = task_name.split('_')[0]

        with self:
            task_success = 'success'

            if init_sim:
                smach.StateMachine.add('InitSimulation', InitSimulation(task_name),
                                       transitions={'success': task_name,
                                                    'fail': 'fail'})

                smach.StateMachine.add('StopSimulation', start_nodes.StopSimulation(),
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
                                                'fail': task_name})



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

        rospy.loginfo('Got yaml %s'%str(self._yaml))
        userdata.yaml = self._yaml
        self._lock.release()

        return 'success'

    def parse_yaml(self, msg):
        self._lock.acquire()
        self._yaml = msg
        rospy.loginfo('Parsed yaml: %s'%str(self._yaml))
        self._lock.release()

