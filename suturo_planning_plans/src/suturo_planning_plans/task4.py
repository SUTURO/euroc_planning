import smach
from suturo_planning_plans.statescanmapmastcam import ScanMapMastCam
from suturo_planning_plans.statescanmaparmmap import ScanMapArmCam

class Task4(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        with self:
            smach.StateMachine.add('ScanMapMastCam', ScanMapMastCam(),
                                   transitions={'mapScanned': 'success'})

            smach.StateMachine.add('ScanMapArmCam', ScanMapArmCam(),
                                   transitions={'mapScanned': 'success'})

        self.userdata.enable_movement = True
        self.userdata.task = task
