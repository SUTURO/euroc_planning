import smach
from suturo_planning_plans.stateSearchForObjects import SearchForObjects
from suturo_planning_plans.stateclassifyobject import ClassifyObjects
from suturo_planning_plans.statescanmapmastcam import ScanMapMastCam
from suturo_planning_plans.statescanmaparmmap import ScanMapArmCam

class Task4(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('ScanMapMastCam', ScanMapMastCam(),
                                   transitions={'mapScanned': 'ScanMapArmCam'})

            smach.StateMachine.add('ScanMapArmCam', ScanMapArmCam(),
                                   transitions={'mapScanned': 'success'})

            # smach.StateMachine.add('SearchForObjects', SearchForObjects(),
            #                        transitions={'mapScanned': 'success'})
            #
            # smach.StateMachine.add('ClassifyObjects', ClassifyObjects(),
            #                        transitions={'objectsClassified': 'FocusObjects',
            #                                     'noObject': 'SearchObject'})
            #
            # smach.StateMachine.add('ChooseObject', ChooseObject(),
            #                        transitions={'objectChosen': 'TidyUpObject',
            #                                     'noObjectsLeft': 'success'})
            #
            # smach.StateMachine.add('TidyUpObject', TidyUpObject(),
            #                        transitions={'success': 'ChooseObject',
            #                                     'fail': 'ChooseObject'})
        self.userdata.enable_movement = True
        self.userdata.task = task
