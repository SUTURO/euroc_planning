from copy import deepcopy
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Vector3 import Vector3
from math import sqrt
from moveit_msgs.msg._CollisionObject import CollisionObject
import smach
import rospy
from suturo_planning_manipulation.mathemagie import magnitude, subtract_point
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position, get_pre_grasp
import utils


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'objectNotInPlanningscene', 'noGraspPosition'],
                             input_keys=['yaml', 'object_to_move', 'enable_movement'],
                             output_keys=['place_position', 'grasp', 'dist_to_obj'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')
        collision_object_name = userdata.object_to_move.mpe_object.id
        rospy.loginfo('Trying to grasp %s' % collision_object_name)

        if userdata.enable_movement:
            move_to_func = utils.manipulation.move_arm_and_base_to
        else:
            move_to_func = utils.manipulation.move_to

        #get the collisionobject out of the planningscene
        collision_object = utils.manipulation.get_planning_scene().get_collision_object(collision_object_name)
        if collision_object is None:
            rospy.logwarn("Collision Object " + collision_object_name + " is not in planningscene.")
            return 'objectNotInPlanningscene'

        grasp_positions = calculate_grasp_position(collision_object, utils.manipulation.transform_to)

        #filter out some invalid grasps
        grasp_positions = utils.manipulation.filter_invalid_grasps(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found for " + collision_object_name)
            return 'noGraspPosition'

        #sort to try the best grasps first
        grasp_positions.sort(cmp=lambda x, y: utils.manipulation.cmp_pose_stamped(collision_object, x, y))
        # visualize_poses(grasp_positions)
        # print grasp_positions

        utils.manipulation.open_gripper()
        for grasp in grasp_positions:
            if move_to_func(get_pre_grasp(grasp)):

                if not move_to_func(grasp):
                    continue

                rospy.sleep(1)
                utils.manipulation.close_gripper(collision_object)

                #calculate the center of mass and weight of the object and call the load object service
                com = utils.manipulation.get_center_of_mass(collision_object)
                com = utils.manipulation.transform_to(com, "/tcp")
                if com is None:
                    rospy.logwarn("TF failed")
                    return False

                density = 1
                for obj in userdata.yaml.objects:
                    if obj.name == collision_object_name:
                        density = obj.primitive_densities[0]

                utils.manipulation.load_object(utils.manipulation.calc_object_weight(collision_object, density),
                                               Vector3(com.point.x, com.point.y, com.point.z))

                rospy.loginfo("grasped " + collision_object_name)

                grasp_2 = utils.manipulation.transform_to(grasp)
                userdata.grasp = grasp_2
                v1 = deepcopy(grasp_2.pose.position)
                v1.z = 0
                v2 = deepcopy(collision_object.primitive_poses[0].position)
                v2.z = 0
                a = magnitude(subtract_point(v1, v2))
                b = abs(grasp_2.pose.position.z - collision_object.primitive_poses[0].position.z)
                c = sqrt(a**2 + b**2)
                userdata.dist_to_obj = abs(c)
                # print c
                #save the grasp for placeposition calculation
                # userdata.grasp = utils.manipulation.make_grasp_vector(collision_object_name)

                rospy.logdebug("lift object")
                if not move_to_func(get_pre_grasp(grasp)):
                    rospy.logwarn("couldnt lift object. continue anyway")

                return 'success'
        rospy.logwarn("Grapsing failed.")
        return 'fail'