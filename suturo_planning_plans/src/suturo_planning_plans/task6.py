import smach
from suturo_planning_plans.statecamtodropzone import CamToDropzone
from suturo_planning_plans.statefastgrasp import FastGrasp
from suturo_planning_plans.stateplacetask6 import PlaceTask6
from suturo_planning_plans.statecheckplacement import CheckPlacement


class Task6(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        # TODO: Fehlerbehandlung
        # TODO: Check auf richtiges Placement einbauen
        with self:
            smach.StateMachine.add('CamToDropzone', CamToDropzone(),
                                   transitions={'scanPoseReached': 'FastGrasp',
                                                'fail': 'fail'})

            smach.StateMachine.add('FastGrasp', FastGrasp(),
                                   transitions={'objectGrasped': 'PlaceObject',
                                                'timeExpired': 'success',
                                                'fail': 'fail'})

            smach.StateMachine.add('PlaceObject', PlaceTask6(),
                                   transitions={'success': 'CamToDropzone',
                                                'fail': 'fail'})

            smach.StateMachine.add('CheckObject', CheckPlacement(),
                                   transitions={'onTarget': 'CamToDropzone',
                                                'notOnTarget': 'fail'})

        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []
        self.userdata.enable_movement = True
        self.userdata.task = task
        self.placement_failed = False