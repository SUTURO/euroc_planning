import smach
from suturo_planning_plans.stateclassifyobject import ClassifyObjects
from suturo_planning_plans.statecamtodropzone import CamToDropzone
from suturo_planning_plans.stategraspobject import GraspObject
from suturo_planning_plans.stateplaceobject import PlaceObject
from suturo_planning_plans.statecheckplacement import CheckPlacement
from suturo_planning_plans.statetask6init import Task6Init


class Task6(smach.StateMachine):
    def __init__(self, task):
        smach.StateMachine.__init__(self, outcomes=['success', 'fail'],
                                    input_keys=['yaml'],
                                    output_keys=[])

        # Foerderband in PS -> Armkamera auf Dropzone -> warten auf Objekt -> Perception triggern ->
        # Wenn Objekt da ist, Manipulation triggern (Berechnung der neuen Pose) -> Pre Grasp -> Graspen bei t_x
        # PlaceObject -> Armkamera auf Dropzone

        with self:
            smach.StateMachine.add('Task6Init', Task6Init(),
                                   transitions={'success': 'success',
                                                'fail': 'fail'})



        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []
        self.userdata.enable_movement = True
        self.userdata.task = task
        self.placement_failed = False