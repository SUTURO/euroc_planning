import smach
from suturo_planning_plans.statechooseobject import ChooseObject
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
                                   transitions={'mapScanned': 'ScanObstacles',
                                                'newImage' : 'ScanMapArmCam'})

            smach.StateMachine.add('ScanObstacles', ScanObstacles(),
                                   transitions={'mapScanned': 'success',
                                                'newImage' : 'ScanMapArmCam'})

            smach.StateMachine.add('SearchObject', SearchObject(),
                                   transitions={'objectFound': 'ClassifyObjects',
                                                'noObjectsLeft': 'ChooseObject',
                                                'simStopped': 'fail'})

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

        self.userdata.enable_movement = True
        self.userdata.task = task
