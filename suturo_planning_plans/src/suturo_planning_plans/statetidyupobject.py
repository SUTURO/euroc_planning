import smach
from suturo_planning_plans.stategraspobject import GraspObject
from suturo_planning_plans.stateplaceobject import PlaceObject
from suturo_planning_plans.statecheckplacement import CheckPlacement


class TidyUpObject(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml', 'enable_movement', 'object_to_move', 'place_position'],
                                    output_keys=['placed_object'])

        with self:
            smach.StateMachine.add('GraspObject', GraspObject(),
                                   transitions={'success': 'PlaceObject',
                                                'objectNotInPlanningscene': 'ChooseObject',
                                                'noGraspPosition': 'ChooseObject',
                                                'fail': 'ChooseObject'})
            smach.StateMachine.add('PlaceObject', PlaceObject(),
                                   transitions={'success': 'CheckPlacement',
                                                'fail': 'ChooseObject',
                                                'noObjectAttached': 'GraspObject',
                                                'noPlacePosition': 'PlaceObject'},
                                   remapping={'target_position': 'place_position'})
            smach.StateMachine.add('CheckPlacement', CheckPlacement(),
                                   transitions={'onTarget': 'ChooseObject',
                                                'notOnTarget': 'ChooseObject'})