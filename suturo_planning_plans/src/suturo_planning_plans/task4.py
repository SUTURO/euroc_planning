import smach
from suturo_planning_plans.statechooseobject import ChooseObject, CleanUpPlan
from suturo_planning_plans.stateclassifyobject import ClassifyObjects
from suturo_planning_plans.statefocusobjects import FocusObject, FocusObjects
from suturo_planning_plans.stateposeestimateobject import PoseEstimateObject
from suturo_planning_plans.statescanmapmastcam import ScanMapMastCam
from suturo_planning_plans.statescanmaparmmap import ScanMapArmCam
from suturo_planning_plans.statescanobstacles import ScanObstacles
from suturo_planning_plans.statesearchobject import SearchObject
from suturo_planning_plans.statetidyupobject import TidyUpObject


class Task4(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('ScanMapMastCam', ScanMapMastCam(),
                                   transitions={'mapScanned': 'ScanMapArmCam'})

            smach.StateMachine.add('ScanMapArmCam', ScanMapArmCam(),
                                   transitions={'mapScanned': 'SearchObject',
                                                'newImage' : 'ScanMapArmCam'})

            smach.StateMachine.add('ScanObstacles', ScanObstacles(),
                                   transitions={'mapScanned': 'CleanUpPlan',
                                                'newImage' : 'ClassifyObjects',
                                                'noRegionLeft' : 'CleanUpPlan'})

            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'searchObject': 'ScanObstacles',
                                                'noObjectsLeft': 'CleanUpPlan',
                                                'simStopped': 'fail'})

            smach.StateMachine.add('ClassifyObjects', ClassifyObjects(),
                                   transitions={'objectsClassified': 'FocusObjects',
                                                'noObject': 'SearchObject'})

            smach.StateMachine.add('FocusObjects', FocusObjects(),
                                   transitions={'success': 'SearchObject',
                                                'focusObject': 'PoseEstimateObject',
                                                'focusHandle': 'PoseEstimateObject',
                                                'fail': 'SearchObject'},
                                   remapping={'objects_to_focus': 'classified_objects'})

            smach.StateMachine.add('PoseEstimateObject', PoseEstimateObject(),
                                   transitions={'success': 'FocusObjects',
                                                'fail': 'FocusObjects'},
                                   remapping={'focused_object': 'object_to_focus'})

            smach.StateMachine.add('CleanUpPlan', CleanUpPlan(),
                                   transitions={'success': 'ChooseObject',
                                                'fail': 'fail'})

            smach.StateMachine.add('ChooseObject', ChooseObject(),
                                   transitions={'objectChosen': 'TidyUpObject',
                                                'noObjectsLeft': 'success'})

            smach.StateMachine.add('TidyUpObject', TidyUpObject(),
                                   transitions={'success': 'ChooseObject',
                                                'fail': 'ChooseObject'})

        self.userdata.objects_found = []
        self.userdata.perceived_objects = []
        self.userdata.fitted_object = None
        self.userdata.fitted_objects = []
        self.userdata.enable_movement = True
        self.userdata.task = task
        self.userdata.cell_coords = []
