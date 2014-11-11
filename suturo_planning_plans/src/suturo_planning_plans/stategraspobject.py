from copy import deepcopy
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Vector3 import Vector3
from math import sqrt
from moveit_msgs.msg._CollisionObject import CollisionObject
import smach
import rospy
import time
from suturo_planning_manipulation.mathemagie import magnitude, subtract_point, get_fingertip
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position, get_pre_grasp
from suturo_planning_visualization.visualization import visualize_poses
import utils
from math import isinf


class GraspObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail', 'objectNotInPlanningscene', 'noGraspPosition'],
                             input_keys=['yaml', 'object_to_move', 'enable_movement'],
                             output_keys=['place_position', 'grasp', 'dist_to_obj', 'failed_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GraspObject')
        collision_object_name = userdata.object_to_move.mpe_object.id
        rospy.loginfo('Trying to grasp %s' % collision_object_name)

        if userdata.enable_movement:
            move_to_func = utils.manipulation.move_arm_and_base_to
            plan_to_func = utils.manipulation.plan_arm_and_base_to
        else:
            move_to_func = utils.manipulation.move_to
            plan_to_func = utils.manipulation.plan_arm_to

        #get the collisionobject out of the planningscene
        collision_object = utils.manipulation.get_planning_scene().get_collision_object(collision_object_name)
        if collision_object is None:
            rospy.logwarn("Collision Object " + collision_object_name + " is not in planningscene.")
            return 'objectNotInPlanningscene'

        rospy.logdebug("Grasping: " + str(collision_object))

        grasp_positions = calculate_grasp_position(collision_object, utils.manipulation.transform_to)

        #filter out some invalid grasps
        grasp_positions = utils.manipulation.filter_low_poses(grasp_positions)

        if not userdata.enable_movement:
            grasp_positions = utils.manipulation.filter_close_poses(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found for " + collision_object_name)
            userdata.failed_object = userdata.object_to_move
            return 'noGraspPosition'

        #sort to try the best grasps first
        grasp_positions.sort(cmp=lambda x, y: utils.manipulation.cmp_pose_stamped(collision_object, x, y))
        visualize_poses(grasp_positions)

        utils.manipulation.open_gripper()
        grasp_positions = [utils.manipulation.transform_to(grasp) for grasp in grasp_positions]
        utils.manipulation.blow_up_objects(do_not_blow_up_list=(collision_object_name))
        for grasp in grasp_positions:
            plan_pre_grasp = plan_to_func(get_pre_grasp(grasp))
            if plan_pre_grasp is None:
                continue

            rospy.logdebug("Plan to pregraspposition found")

            utils.manipulation.blow_up_objects(do_not_blow_up_list=("map", collision_object_name))
            plan_to_grasp = plan_to_func(grasp, start_state=utils.manipulation.get_end_state(plan_pre_grasp))
            if plan_to_grasp is None or not utils.manipulation.move_with_plan_to(plan_pre_grasp):
                rospy.logdebug("Failed to move to Graspposition")
                continue
            rospy.sleep(0.5)
            if not utils.manipulation.move_with_plan_to(plan_to_grasp):
                rospy.logdebug("Failed to move to Graspposition")
                continue

            rospy.logdebug("Graspposition taken")

            time.sleep(0.5)
            rospy.sleep(1)
            if not utils.manipulation.close_gripper(collision_object, get_fingertip(utils.manipulation.transform_to(grasp))):
                # userdata.failed_object = userdata.object_to_move
                utils.manipulation.blow_down_objects()
                return 'fail'
            time.sleep(0.5)
            rospy.sleep(1.5)

            #calculate the center of mass and weight of the object and call the load object service
            com = utils.manipulation.get_center_of_mass(collision_object)
            com = utils.manipulation.transform_to(com, "/tcp")
            if com is None:
                rospy.logwarn("TF failed")
                # userdata.failed_object = userdata.object_to_move
                utils.manipulation.blow_down_objects()
                return 'fail'

            density = 1
            for obj in userdata.yaml.objects:
                if obj.name == collision_object_name:
                    density = obj.primitive_densities[0]
                    if isinf(density):
                        rospy.logerr("Infinite density! WTF. setting density to 7850")
                        density = 7850

            utils.manipulation.load_object(utils.manipulation.calc_object_weight(collision_object, density),
                                           Vector3(com.point.x, com.point.y, com.point.z))

            rospy.loginfo("grasped " + collision_object_name)

            #save grasp data for placing
            userdata.grasp = grasp
            fingertip = get_fingertip(grasp)
            fingertip_to_tcp = subtract_point(grasp.pose.position, fingertip.point)
            userdata.dist_to_obj = magnitude(fingertip_to_tcp)

            rospy.logdebug("lift object")
            the_pre_grasp = get_pre_grasp(grasp)
            the_move_to_func = move_to_func(the_pre_grasp, do_not_blow_up_list=collision_object_name)

            if not the_move_to_func:
                rospy.logwarn("couldnt lift object. continue anyway")
            userdata.failed_object = None
            return 'success'
        rospy.logwarn("Grapsing failed.")
        userdata.failed_object = userdata.object_to_move
        utils.manipulation.blow_down_objects()
        return 'fail'