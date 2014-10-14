import smach


class CheckPlacement(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['onTarget', 'notOnTarget'],
                             input_keys=['yaml', 'object_to_move', 'pending_objects'],
                             output_keys=['placed_object'])

    def execute(self, userdata):

        userdata.placed_object = userdata.object_to_move.mpe_object.id

        return 'onTarget'