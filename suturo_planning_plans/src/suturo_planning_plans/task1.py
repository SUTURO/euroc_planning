import smach
from suturo_planning_plans.stateclassifyobject import ClassifyObjects
from suturo_planning_plans.statesearchobject import SearchObject
from suturo_planning_plans.statetidyupobject import TidyUpObject
from suturo_planning_plans.statechooseobject import ChooseObject
from suturo_planning_plans.statefocusobjects import FocusObjects, FocusObject
from suturo_planning_plans.stateposeestimateobject import PoseEstimateObject


class Task1(smach.StateMachine):
    def __init__(self, enable_movement, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'ClassifyObjects',
                                                'noObjectsLeft': 'ChooseObject'})
            smach.StateMachine.add('ClassifyObjects', ClassifyObjects(),
                                   transitions={'objectsClassified': 'FocusObjects',
                                                'noObject': 'SearchObject'})
            smach.StateMachine.add('FocusObjects', FocusObjects(),
                                   transitions={'success': 'SearchObject',
                                                'nextObject': 'FocusObject',
                                                'fail': 'SearchObject'},
                                   remapping={'objects_to_focus': 'classified_objects'})
            smach.StateMachine.add('FocusObject', FocusObject(),
                                   transitions={'success': 'PoseEstimateObject',
                                                'fail': 'FocusObjects'},
                                   remapping={'objects_to_focus': 'classified_objects'})
            smach.StateMachine.add('PoseEstimateObject', PoseEstimateObject(),
                                   transitions={'success': 'FocusObjects',
                                                'fail': 'FocusObjects'})
            smach.StateMachine.add('ChooseObject', ChooseObject(),
                                   transitions={'objectChosen': 'TidyUpObject',
                                                'noObjectsLeft': 'success'})
            smach.StateMachine.add('TidyUpObject', TidyUpObject(),
                                   transitions={'success': 'ChooseObject',
                                                'fail': 'ChooseObject'})

        self.userdata.objects_found = []
        self.userdata.perceived_objects = []
        self.userdata.fitted_object = None
        self.userdata.enable_movement = enable_movement
        self.userdata.task = task