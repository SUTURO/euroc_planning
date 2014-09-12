import smach
from suturo_planning_plans.stateperceiveobject import PerceiveObject
from suturo_planning_plans.statesearchobject import SearchObject
from suturo_planning_plans.stategraspobject import GraspObject
from suturo_planning_plans.stateplaceobject import PlaceObject
from suturo_planning_plans.statecheckplacement import CheckPlacement


class Task3(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'PerceiveObject',
                                                'noObjectsLeft': 'success'})
            smach.StateMachine.add('PerceiveObject', PerceiveObject(),
                                   transitions={'validObject': 'GraspObject',
                                                'noObject': 'SearchObject'})
            smach.StateMachine.add('GraspObject', GraspObject(),
                                   transitions={'success': 'PlaceObject',
                                                'fail': 'GraspObject'})
            smach.StateMachine.add('PlaceObject', PlaceObject(),
                                   transitions={'success': 'CheckPlacement',
                                                'fail': 'PerceiveObject'})
            smach.StateMachine.add('CheckPlacement', CheckPlacement(),
                                   transitions={'onTarget': 'SearchObject',
                                                'notOnTarget': 'GraspObject',
                                                'nextObject': 'GraspObject'})

        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []