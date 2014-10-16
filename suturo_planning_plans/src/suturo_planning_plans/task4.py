import smach
from suturo_planning_plans.stateperceiveobject import PerceiveObject
from suturo_planning_plans.statescanmaparmmap import ScanMapArmCam
from suturo_planning_plans.statescanmapmastcam import ScanMapMastCam
from suturo_planning_plans.statesearchobject import SearchObject
from suturo_planning_plans.stategraspobject import GraspObject
from suturo_planning_plans.stateplaceobject import PlaceObject
from suturo_planning_plans.statecheckplacement import CheckPlacement
from suturo_planning_plans.statechooseobject import ChooseObject


class Task4(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('ScanMapMastCam', ScanMapMastCam(),
                                   transitions={'mapScanned': 'success'})

            smach.StateMachine.add('ScanMapArmCam', ScanMapArmCam(),
                                   transitions={'mapScanned': 'success'},
                                   remapping={'oldmap': 'map'})

            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'PerceiveObject',
                                                'noObjectsLeft': 'success'})

            smach.StateMachine.add('PerceiveObject', PerceiveObject(),
                                   transitions={'objectsPerceived': 'ChooseObject',
                                                'noObject': 'SearchObject'})

            smach.StateMachine.add('ChooseObject', ChooseObject(),
                                   transitions={'objectChosen': 'GraspObject',
                                                'noObjectsLeft': 'SearchObject'})

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

        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []
        self.userdata.enable_movement = True
        self.userdata.task = task
        self.placement_failed = False