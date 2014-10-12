import smach
from suturo_planning_plans.stateperceiveobject import PerceiveObject
from suturo_planning_plans.statesearchobject import SearchObject
from suturo_planning_plans.statetidyupobject import TidyUpObject
from suturo_planning_plans.statechooseobject import ChooseObject


class Task1(smach.StateMachine):
    def __init__(self, enable_movement, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'PerceiveObject',
                                                'noObjectsLeft': 'ChooseObject'})
            smach.StateMachine.add('PerceiveObject', PerceiveObject(),
                                   transitions={'objectsPerceived': 'SearchObject',
                                                'noObject': 'SearchObject'})
            smach.StateMachine.add('ChooseObject', ChooseObject(),
                                   transitions={'objectChosen': 'TidyUpObject',
                                                'noObjectsLeft': 'success'})
            smach.StateMachine.add('TidyUpObject', TidyUpObject(),
                                   transitions={'success': 'ChooseObject',
                                                'fail': 'ChooseObject'})

        self.userdata.objects_found = []
        self.userdata.perceived_objects = []
        self.userdata.placed_objects = []
        self.userdata.enable_movement = enable_movement
        self.userdata.task = task