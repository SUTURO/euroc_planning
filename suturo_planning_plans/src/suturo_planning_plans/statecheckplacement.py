import smach


class CheckPlacement(smach.State):

    _placed_objects = []

    def __init__(self):
        smach.State.__init__(self, outcomes=['onTarget', 'notOnTarget'],
                             input_keys=['yaml', 'object_to_move', 'pending_objects'],
                             output_keys=['placed_objects'])

    def execute(self, userdata):

        self._placed_objects.append(userdata.object_to_move.mpe_object.id)
        userdata.placed_objects = self._placed_objects

        return 'onTarget'