from copy import deepcopy
from math import isinf
from geometry_msgs.msg import Vector3, PoseStamped
import rospy
from suturo_planning_interface import utils
from suturo_planning_manipulation.calc_grasp_position import calculate_grasp_position, get_pre_grasp
from suturo_planning_visualization.visualization import visualize_poses
from mathemagie import get_fingertip, subtract_point, magnitude
import time
from suturo_perception_msgs.msg import EurocObject
from suturo_manipulation_msgs.srv import GraspObject as GraspObjectService
from suturo_manipulation_msgs.srv import GraspObjectResponse, GraspObjectRequest
from suturo_planning_manipulation.manipulation import Manipulation

class GraspObject(object):
    SERVICE_NAME = 'suturo/manipulation/grasp_object'

    def __init__(self):
        self.grasp_position = PoseStamped()
        self._create_service()

    def _create_service(self):
        rospy.Service(self.SERVICE_NAME, GraspObjectService, self.__handle_request)

    def __handle_request(self, req):
        result = self.__grasp(req.object, req.density, req.prefer_grasp_position)
        return GraspObjectResponse(result=result, grasp_position=self.grasp_position)

    def __grasp(self, collision_object, density, prefer_grasp_position):
        rospy.loginfo('Executing state GraspObject')

        if utils.manipulation is None:
            utils.manipulation = Manipulation()

        move_to_func = utils.manipulation.move_to
        plan_to_func = utils.manipulation.plan_arm_to

        collision_object = utils.manipulation.get_planning_scene().get_collision_object(collision_object.id)
        if collision_object is None:
            rospy.logwarn("Collision Object is not in planningscene.")
            return False

        rospy.logdebug("Grasping: " + str(collision_object))

        grasp_positions = calculate_grasp_position(collision_object, utils.manipulation.transform_to) #PoseStamped[]

        #filter out some invalid grasps
        grasp_positions = utils.manipulation.filter_low_poses(grasp_positions)

        ## Only do this when the base cannot move
        grasp_positions = utils.manipulation.filter_close_poses(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found")

        # rospy.logwarn("printing found")
        for position in grasp_positions:
            rospy.logdebug("%s", position)


        #sort to try the best grasps first
        if prefer_grasp_position == 1:
            grasp_positions = filter(lambda pos: pos.pose.orientation.y >= 0.67 and pos.pose.orientation.y <= 0.73 and
                    pos.pose.orientation.w >= 0.67 and pos.pose.orientation.w <= 0.73 and pos.pose.orientation.x == 0 and
                    pos.pose.orientation.z == 0, grasp_positions)
        elif prefer_grasp_position == 2:
            #grasp_positions = filter(filterSide, grasp_positions)
            pass

        # rospy.logwarn("printing filtered")
        for position in grasp_positions:
            rospy.logdebug("%s", position)

        grasp_positions.sort(cmp=lambda x, y: utils.manipulation.cmp_pose_stamped(collision_object, x, y))

        # rospy.logwarn("printing sorted")
        for position in grasp_positions:
            rospy.logdebug("%s", position)
        visualize_poses(grasp_positions)

        utils.manipulation.open_gripper()
        grasp_positions = [utils.manipulation.transform_to(grasp) for grasp in grasp_positions]
        # rospy.logwarn("printing transformed position")
        for position in grasp_positions:
            rospy.logdebug("%s", position)

        utils.manipulation.blow_up_objects(do_not_blow_up_list=collision_object.id)
        for grasp in grasp_positions:
            self.grasp_position = grasp
            plan_pre_grasp = plan_to_func(get_pre_grasp(grasp))
            if plan_pre_grasp is None:
                continue

            rospy.logdebug("Plan to pregraspposition found")

            utils.manipulation.blow_up_objects(do_not_blow_up_list=("map", collision_object.id))
            plan_to_grasp = plan_to_func(grasp, start_state=utils.manipulation.get_end_state(plan_pre_grasp))
            if plan_to_grasp is None or not utils.manipulation.move_with_plan_to(plan_pre_grasp):
                rospy.logwarn("Failed to move to Graspposition pre grasp:  %s", plan_to_grasp)
                rospy.logdebug("Failed to move to Graspposition")
                continue
            rospy.sleep(0.5)
            if not utils.manipulation.move_with_plan_to(plan_to_grasp):
                rospy.logwarn("Failed to move to Graspposition")
                rospy.logdebug("Failed to move to Graspposition")
                continue

            rospy.logdebug("Graspposition taken")

            time.sleep(0.5)
            rospy.sleep(1)
            if not utils.manipulation.close_gripper(collision_object, get_fingertip(utils.manipulation.transform_to(grasp))):
                # taskdata.failed_object = taskdata.object_to_move
                utils.manipulation.blow_down_objects()
                return False
            time.sleep(0.5)
            rospy.sleep(1.5)

            #calculate the center of mass and weight of the object and call the load object service
            com = utils.manipulation.get_center_of_mass(collision_object)
            com = utils.manipulation.transform_to(com, "/tcp")
            if com is None:
                rospy.logwarn("TF failed")
                # taskdata.failed_object = taskdata.object_to_move
                utils.manipulation.blow_down_objects()
                return False

            if isinf(density):
                rospy.logerr("Infinite density! WTF. setting density to 7850")
                density = 7850

            utils.manipulation.load_object(utils.manipulation.calc_object_weight(collision_object, density),
                                           Vector3(com.point.x, com.point.y, com.point.z))

            rospy.loginfo("grasped")

            #save grasp data for placing
            fingertip = get_fingertip(grasp)
            fingertip_to_tcp = subtract_point(grasp.pose.position, fingertip.point)

            rospy.logdebug("lift object")
            if collision_object.id == "blue_handle":
                the_pre_grasp = deepcopy(grasp)
                the_pre_grasp.pose.position.z += 0.1
            else:
                the_pre_grasp = get_pre_grasp(grasp)
            the_move_to_func = move_to_func(the_pre_grasp, do_not_blow_up_list=collision_object.id)

            if not the_move_to_func:
                rospy.logwarn("couldnt lift object. continue anyway")

            return True
        rospy.logwarn("Grapsing failed.")
        utils.manipulation.blow_down_objects()
        return False

