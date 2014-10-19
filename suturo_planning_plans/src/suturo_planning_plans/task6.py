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
                                   transitions={'success': CamToDropzone,
                                                'fail': 'fail'})

            # TODO: Was tun wenn das failed? Wie stell ich fest, dass wir durch sind?
            smach.StateMachine.add('CamToDropzone', CamToDropzone(),
                                   transitions={'success': 'ClassifyObjects',
                                                'fail': 'success'})

            # TODO: Brauchen wir das Objekt in der Planningscene
            smach.StateMachine.add('ClassifyObjects', ClassifyObjects(),
                                   transitions={'objectsClassified': 'GraspObject',
                                                'noObject': 'fail'})

            # Wird zu GraspObjectTask6 und ueberlegen, was bei den einzelnen Dingen genau passieren soll (Benny)
            smach.StateMachine.add('GraspObject', GraspObject(),
                                   transitions={'success': 'PlaceObject',
                                                'objectNotInPlanningscene': 'ClassifyObjects',
                                                'fail': 'ChooseObject'})

            smach.StateMachine.add('PlaceObject', PlaceObject(),
                                   transitions={'success': 'CheckPlacement',
                                                'fail': 'ChooseObject',
                                                'noObjectAttached': 'GraspObject',
                                                'noPlacePosition': 'PlaceObject'},
                                   remapping={'target_position': 'place_position'})

            # Ist das zwingend noetig?
            smach.StateMachine.add('CheckPlacement', CheckPlacement(),
                                   transitions={'onTarget': 'CamToDropzone',
                                                'notOnTarget': 'GraspObject'})

        self.userdata.objects_found = []
        self.userdata.pending_objects = []
        self.userdata.placed_objects = []
        self.userdata.enable_movement = True
        self.userdata.task = task
        self.placement_failed = False