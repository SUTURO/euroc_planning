__author__ = 'ichumuh'

#actual length is 0.06, 0.04  to graps a little closer
finger_length = 0.04

#actual length is 0.173, 0.183 for 1cm puffer between grapsed object and hand
hand_length = 0.183
pre_grasp_length = 0.1

pre_place_length = 0.06
post_place_length = 0.1
safe_place = 0.005

gripper_max_pose = 0.0345

max_motion_time = 40

MOVE_SERVICE = "/suturo/manipulation/move"
MOVE_RELATIVE_SERVICE= "/suturo/manipulation/move_relative"
PLAN_SERVICE = "/suturo/manipulation/plan"
MOVE_WITH_PLAN_SERVICE = "/suturo/manipulation/move_with_plan"
ADD_COLLISION_OBJECTS_SERVICE = "/suturo/manipulation/add_collision_objects"
GET_COLLSISION_OBJECT_SERVICE = "/suturo/manipulation/get_collision_object"
MOVE_MASTCAM_SERVICE = "/suturo/manipulation/move_mastcam"
OPEN_GRIPPER_SERVICE = "/suturo/manipulation/open_gripper"
CLOSE_GRIPPER_SERVICE = "/suturo/manipulation/close_gripper"
CREATE_POSES_FOR_OBJECT_SCANNING_SERVICE = "/suturo/environment/create_poses_for_object_scanning"
BLOW_UP_OBJECTS_SERVICE = "/suturo/manipulation/blow_up_objects"
BLOW_DOWN_OBJECTS_SERVICE = "/suturo/manipulation/blow_down_objects"
GET_EEF_POSITION_TOPIC = "/suturo/manipulation/get_eef_position"
BASE_ORIGIN_TOPIC = "/suturo/environment/get_base_origin"


