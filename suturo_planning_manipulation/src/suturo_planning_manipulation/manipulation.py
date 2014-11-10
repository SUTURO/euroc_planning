#!/usr/bin/env python
from Crypto.Random.OSRNG import posix
from asyncore import dispatcher
from docutils.parsers.rst.roles import role
from math import pi
from math import isnan
from pdb import post_mortem

import sys
import copy
from euroc_c2_msgs.srv import *
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._Vector3 import Vector3
from moveit_msgs.msg import *
from moveit_msgs.srv._GetMotionPlan import GetMotionPlan, GetMotionPlanRequest
import geometry_msgs.msg
from std_msgs.msg._Duration import Duration
from calc_grasp_position import *
from place import get_place_position, get_pre_place_position, get_grasped_part
from planningsceneinterface import *
from manipulation_constants import *
from manipulation_service import *
import math
# from suturo_planning_visualization.visualization import visualize_poses
from suturo_planning_visualization.visualization import visualize_poses, visualize_pose
from transformer import Transformer
from euroc_c2_msgs.msg import *
from euroc_c2_msgs.srv import *
from sensor_msgs.msg import JointState
from shape_msgs.msg._SolidPrimitive import SolidPrimitive
from suturo_msgs.msg import Task
import time


class Manipulation(object):
    def __init__(self, yaml=None):
        self.tf = Transformer()

        moveit_commander.roscpp_initialize(sys.argv)
        self.__arm_group = moveit_commander.MoveGroupCommander("arm")
        self.__arm_group.set_planning_time(5)

        self.__gripper_group = moveit_commander.MoveGroupCommander("gripper")

        self.__base_group = moveit_commander.MoveGroupCommander("base")
        self.__base_group.set_planning_time(2.5)

        self.__arm_base_group = moveit_commander.MoveGroupCommander("arm_base")
        self.__arm_base_group.set_planning_time(10)

        rospy.wait_for_service('/plan_kinematic_path')
        self.__plan_service = rospy.ServiceProxy('/plan_kinematic_path', GetMotionPlan)

        self.__planning_scene_interface = PlanningSceneInterface()

        euroc_interface_node = '/euroc_interface_node/'
        self.__set_object_load_srv = rospy.ServiceProxy(euroc_interface_node + 'set_object_load', SetObjectLoad)

        self.__manService = ManipulationService()

        rospy.sleep(1)
        self.__planning_scene_interface.add_yaml(yaml)
        # self.__planning_scene_interface.add_ground()
        #
        # self.__planning_scene_interface.add_cam_mast()

        self.__grasp = None

        rospy.loginfo("Manipulation started.")

    def __del__(self):
        # self.print_manipulation()
        if moveit_commander is not None:
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)

    def print_manipulation(self):
        # print "current joint state"
        # print self.get_current_joint_state()
        # print "current planning scene"
        # print self.get_planning_scene().get_planning_scene()
        pass

    def move_base(self, goal_pose):
        """
        Moves the arm's base to the goal position. (Don't use this for Task 1 and 2)
        :param goal_pose: goal position as a PoseStamped
        :return: success of the movement
        """
        goal = [goal_pose.pose.position.x, goal_pose.pose.position.y]
        rospy.logdebug("Move base to: " + str(goal))
        m = self.__base_group.get_current_joint_values()
        d1 = goal[0] - m[0]
        d2 = goal[1] - m[1]
        if 0 <= abs(d1) <= 0.01 and 0 <= abs(d2) <= 0.01:
            rospy.loginfo("No movement required.")
            return True
        self.__base_group.set_joint_value_target(goal)
        path = self.__base_group.plan()
        return self.__manService.move(path)

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        """
        Transforms the pose_target into the target_frame.
        :param pose_target: object to transform as PoseStamped/PointStamped/Vector3Stamped/CollisionObject/PointCloud2
        :param target_frame: goal frame id
        :return: transformed object
        """
        return self.tf.transform_to(pose_target, target_frame)

    def move_to(self, goal_pose, blow_up=()):
        """
        Moves the endeffector to the goal position, without moving the base.
        :param goal_pose: goal position as PoseStamped
        :return: success of the movement
        """
        rospy.logdebug("move_to called!")
        return self.__move_group_to(goal_pose, self.__arm_group, blow_up)

    def move_arm_and_base_to(self, goal_pose, blow_up=()):
        """
        Moves the endeffector to the goal position. (Don't use this for Task 1 and 2)
        :param goal_pose: goal position as PoseStamped
        :return: success of the movement
        """
        # print("move_arm_and_base_to called!")
        # rospy.logdebug("move_arm_and_base_to called!")
        if (math.isnan(goal_pose.pose.orientation.x) or
                math.isnan(goal_pose.pose.orientation.y) or
                math.isnan(goal_pose.pose.orientation.z) or
                math.isnan(goal_pose.pose.orientation.w)):
            rospy.loginfo('move_arm_and_base to goal pose with nan in orientation!')
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 1.0
        # rospy.logdebug("move_arm_and_base_to goal_pose = " + str(goal_pose) + ", blow_up = "+str(blow_up))
        # print("move_arm_and_base_to goal_pose = " + str(goal_pose) + ", blow_up = "+str(blow_up))
        return self.__move_group_to(goal_pose, self.__arm_base_group, blow_up)

    def __move_group_to(self, goal_pose, move_group, blow_up, blow_up_distance=0.02):
        """
         :param goal_pose: the pose which shall be arrived
         :param move_group: the move group which shall be moved
         :param blow_up: True if collision objects shall be made bigger
         :param blow_up_distance: Distance in m
         :return:
         """
        #rospy.logdebug("__move_group_to called!")
        #rospy.logdebug("__move_group_to.goal_pose = "+str(goal_pose)+" .move_group = "+str(move_group)+" .blow_up = "+str(blow_up))
        path = self.__plan_group_to(goal_pose, move_group, blow_up, None)
        #rospy.logdebug("__move_group_to got a path: "+str(path))
        ret = self.move_with_plan_to(path)
        #rospy.logdebug("__move_group_to return value = "+str(ret))
        return ret

    def move_with_plan_to(self, plan):
        #rospy.logdebug("move_with_plan_to called!")
        #rospy.logdebug("move_with_plan_to.plan = "+str(plan))
        if plan is None:
            #rospy.logdebug("move_with_plan_to plan is None")
            return False
        if type(plan) is RobotTrajectory:
            #rospy.logdebug("move_with_plan_to plan is RobotTrajectory")
            return self.__manService.move(plan)
        #rospy.logdebug("plan is neither None nor RobotTrajectory")
        return self.__manService.move(plan.motion_plan_response.trajectory)

    def plan_arm_to(self, goal_pose, blow_up=(), start_state=None):
        return self.__plan_group_to(goal_pose, self.__arm_group, blow_up, start_state)

    def plan_arm_and_base_to(self, goal_pose, blow_up=(), start_state=None):
        return self.__plan_group_to(goal_pose, self.__arm_base_group, blow_up, start_state)

    def __plan_group_to(self, goal_pose, move_group, blow_up, start_state, blow_up_distance=0.015):
        #rospy.logdebug("__plan_group_to called!")
        original_objects = self.__planning_scene_interface.get_collision_objects()
        if not blow_up is None and "all" not in blow_up:
            for each in original_objects:
                if each.id in blow_up:
                    continue
                if not each.id in self.__planning_scene_interface.safe_objects:
                    if each.id == "map":
                        bobj = self.__blow_up_map(each)
                    else:
                        bobj = self.__blow_up_object(each, blow_up_distance)
                    self.__planning_scene_interface.add_object(bobj)
            rospy.sleep(1.5)
        # elif blow_up == 2:
        # print "muh"
        #     map = self.__planning_scene_interface.get_collision_object("map")
        #     map = self.__blow_up_map(map)
        #     self.__planning_scene_interface.add_object(map)
        #     rospy.sleep(1.5)

        move_group.set_start_state_to_current_state()
        goal = deepcopy(goal_pose)

        #rospy.logdebug("goal = " + str(goal))
        if type(goal) is str:
            #rospy.logdebug("move_group.set_named_target")
            move_group.set_named_target(goal)
            # rospy.logwarn("DANGER, for named targets, attached objects will be ignored.")
            plan = move_group.plan()
            if blow_up != 0:
                self.__planning_scene_interface.add_objects(original_objects)
            return plan
        elif type(goal) is PoseStamped:
            #rospy.logdebug("goal is pose stamped")
            visualize_pose(goal)
            # Rotate the goal so that the gripper points from 0,0,0 to 1,0,0 with a 0,0,0,1 quaternion as orientation.
            goal.pose.orientation = rotate_quaternion(goal.pose.orientation, pi / 2, pi, pi / 2)
            # rospy.logdebug("goal after rotation: " + str(goal))
            if goal.header.frame_id != "/odom_combined":
                # rospy.logdebug("goal after transformation: " + str(goal))
                goal = self.tf.transform_to(goal)

                # move_group.set_pose_target(goal)
        # else:
        #     move_group.set_joint_value_target(goal)

        plan = self.plan(move_group, goal, start_state)
        if not plan is None:
            plan2 = self.plan(move_group, goal, self.get_end_state(plan))
            plan.motion_plan_response.trajectory.joint_trajectory.points.extend(
                plan2.motion_plan_response.trajectory.joint_trajectory.points[1:])

        if blow_up != 0:
            self.__planning_scene_interface.add_objects(original_objects)

        #rospy.logdebug("_plan_group_to done, return value: "+str(plan))
        return plan

    def plan(self, move_group, goal, start_state):
        request = GetMotionPlanRequest()
        if start_state is None:
            request.motion_plan_request.start_state.is_diff = True
        else:
            request.motion_plan_request.start_state = start_state

        request.motion_plan_request.allowed_planning_time = move_group.get_planning_time()
        request.motion_plan_request.group_name = move_group.get_name()
        request.motion_plan_request.num_planning_attempts = 1

        constraint = Constraints()
        constraint.name = "muh23"

        if type(goal) is PoseStamped:
            pose_goal = self.__make_position_goal(move_group, goal)
            constraint.position_constraints.append(pose_goal[0])
            constraint.orientation_constraints.append(pose_goal[1])
        else:
            joint_goal = self.__make_joint_state_goal(move_group, goal)
            constraint.joint_constraints.extend(joint_goal)

        request.motion_plan_request.goal_constraints.append(constraint)

        request.motion_plan_request.planner_id = ""

        try:
            resp = self.__plan_service(request)
            return resp
        except rospy.ServiceException as exc:
            rospy.logdebug("Service did not process request: " + str(exc))
            rospy.logdebug("probably couldnt find a plan.")
            return None

    def __make_joint_state_goal(self, move_group, goal):
        joint_goals = []
        joint_names = []
        if len(goal) == 7:
            joint_names = self.get_arm_move_group().get_joints()
        elif len(goal) == 9:
            joint_names = self.get_arm_base_move_group().get_joints()
        joint_names = [name for name in joint_names if name != "base_joint"]
        if len(goal) != len(joint_names):
            rospy.logwarn("length of joints does not equal length of joint names")

        for i in xrange(len(goal)):
            joint_goal = JointConstraint()

            joint_goal.joint_name = joint_names[i]
            joint_goal.position = goal[i]
            joint_goal.tolerance_above = 0.0001
            joint_goal.tolerance_above = 0.0001

            joint_goal.weight = 1.0
            joint_goals.append(joint_goal)

        return joint_goals

    def __make_position_goal(self, move_group, goal):

        position_tolerance = 0.00001
        orientation_tolerance = 0.001

        position_goal = PositionConstraint()
        position_goal.header = goal.header
        position_goal.link_name = move_group.get_end_effector_link()
        position_goal.target_point_offset = Vector3()

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions.append(position_tolerance)

        position_goal.constraint_region.primitives.append(primitive)
        p = Pose()
        p.position.x = goal.pose.position.x
        p.position.y = goal.pose.position.y
        p.position.z = goal.pose.position.z
        p.orientation.w = 1
        position_goal.constraint_region.primitive_poses.append(p)
        position_goal.weight = 1.0

        orientation_goal = OrientationConstraint()
        orientation_goal.header = goal.header
        orientation_goal.link_name = move_group.get_end_effector_link()
        orientation_goal.absolute_x_axis_tolerance = orientation_tolerance
        orientation_goal.absolute_y_axis_tolerance = orientation_tolerance
        orientation_goal.absolute_z_axis_tolerance = orientation_tolerance
        orientation_goal.orientation = goal.pose.orientation
        orientation_goal.weight = 1.0
        return (position_goal, orientation_goal)

    def get_base_origin(self):
        """
        :return: The centre of the arm's base as PointStamped
        """
        current_pose = self.__base_group.get_current_joint_values()
        result = PointStamped()
        result.header.frame_id = "/odom_combined"
        result.point = Point(current_pose[0], current_pose[1], 0)
        return result

    def get_eef_position(self):
        """
        :return: The centre of the arm's base as PointStamped
        """
        current_pose = self.__arm_group.get_current_pose()
        return current_pose

    def __blow_up_object(self, bobject, factor):
        """
        :param bobject: Object to blow up
        :param factor: Blowup Factor
        :return: Returns the Blown up object
        """
        o = deepcopy(bobject)
        for primitive in o.primitives:
            dims = []
            for dimension in primitive.dimensions:
                dims.append(dimension + factor)
            primitive.dimensions = dims
        return o

    def __blow_up_map(self, object):
        o = deepcopy(object)
        for primitive in o.primitives:
            dim = []
            dim.append(primitive.dimensions[0] + 0.005)
            dim.append(primitive.dimensions[1] + 0.005)
            dim.append(primitive.dimensions[2] + 0.175)
            primitive.dimensions = dim
        return o

    def get_end_state(self, plan_response):
        # if type(plan_response) is RobotTrajectory:
        # r = RobotTrajectory()
        #     robot_state = RobotState()
        #     robot_state.joint_state.header = r.

        r = plan_response
        robot_state = RobotState()
        robot_state.multi_dof_joint_state = r.motion_plan_response.trajectory_start.multi_dof_joint_state

        robot_state.joint_state.header = r.motion_plan_response.trajectory.joint_trajectory.header
        robot_state.joint_state.name = r.motion_plan_response.trajectory.joint_trajectory.joint_names
        robot_state.joint_state.position = r.motion_plan_response.trajectory.joint_trajectory.points[-1].positions
        robot_state.joint_state.velocity = r.motion_plan_response.trajectory.joint_trajectory.points[-1].velocities
        robot_state.joint_state.effort = r.motion_plan_response.trajectory.joint_trajectory.points[-1].effort
        robot_state.attached_collision_objects = r.motion_plan_response.trajectory_start.attached_collision_objects
        return robot_state

    def get_current_joint_state(self):
        """
        :return: current joint state as list of floats
        """
        return self.__arm_base_group.get_current_joint_values()

    def get_current_gripper_state(self):
        """
        :return: current joint state as list of floats
        """
        return self.__gripper_group.get_current_joint_values()

    def get_current_lwr_joint_state(self):
        return self.__arm_group.get_current_joint_values()

    def open_gripper(self, position=gripper_max_pose):
        """
        Opens the gripper and detaches any attached collisionobject.
        :param position: the desired finger position, max value if not specified.
        :return: success of the movement
        """

        self.__gripper_group.set_joint_value_target([-position, position])
        path = self.__gripper_group.plan()
        if self.__manService.move(path):
            self.__gripper_group.detach_object()
            self.load_object(0, Vector3(0, 0, 0))
            rospy.logdebug("Gripper opened")
            return True
        else:
            rospy.logdebug("Gripper failed to open")
            return False

    def close_gripper(self, object=None, grasp_point=None):
        """
        Closes the gripper completely or far enough to hold the object, when one is given
        :param object: Object that will be grasped.
        :return: success of the movement
        """

        rospy.logdebug("Closing Gripper")
        if type(object) is CollisionObject:
            self.__gripper_group.attach_object(object.id, "gp", ["gp", "finger1", "finger2"])
            rospy.sleep(1.0)
            id = get_grasped_part(object, grasp_point)[1]
            # id = min(range(len(object.primitives)), key=lambda i: min(object.primitives[i].dimensions))
            # TODO: only works for cubes and cylinders and only "sometimes" for object compositions
            if object.primitives[id].type == shape_msgs.msg.SolidPrimitive.BOX:
                length = min(object.primitives[id].dimensions)
                self.__gripper_group.set_joint_value_target([-(length / 2), length / 2])
            elif object.primitives[id].type == shape_msgs.msg.SolidPrimitive.CYLINDER:
                radius = object.primitives[id].dimensions[shape_msgs.msg.SolidPrimitive.CYLINDER_RADIUS]
                if radius >= gripper_max_pose:
                    rospy.logdebug("Object is too big!")
                    return False
                self.__gripper_group.set_joint_value_target([-radius + 0.005, radius - 0.005])
        else:
            self.__gripper_group.set_joint_value_target([0.0, 0.0])
        path = self.__gripper_group.plan()
        return self.__manService.move(path)

    def grasp(self, collision_object, object_density=1):
        """
        Deprecated. For testing only
        """
        return self.__grasp_with_group(collision_object, self.__arm_group, object_density)

    def grasp_and_move(self, collision_object, object_density=1):
        """
        Deprecated. For testing only
        """
        return self.__grasp_with_group(collision_object, self.__arm_base_group, object_density)

    def __grasp_with_group(self, collision_object_name, move_group, object_density):
        """
        Deprecated. For testing only
        """
        if type(collision_object_name) is CollisionObject:
            collision_object_name = collision_object_name.id

        collision_object = self.__planning_scene_interface.get_collision_object(collision_object_name)
        if collision_object is None:
            rospy.logwarn("Collision Object " + collision_object_name + " is not in planningscene.")
            return False

        grasp_positions = calculate_grasp_position(collision_object, self.tf.transform_to)

        grasp_positions = self.filter_low_poses(grasp_positions)
        grasp_positions = self.filter_close_poses(grasp_positions)

        if len(grasp_positions) == 0:
            rospy.logwarn("No grasppositions found for " + collision_object_name)

        grasp_positions.sort(cmp=lambda x, y: self.cmp_pose_stamped(collision_object, x, y))
        visualize_poses(grasp_positions)
        # print grasp_positions

        self.open_gripper()
        for grasp in grasp_positions:
            if self.__move_group_to(get_pre_grasp(self.transform_to(grasp)), move_group, blow_up=("map", collision_object_name)):

                if not self.__move_group_to(grasp, move_group, blow_up=("map", collision_object_name)):
                    continue
                rospy.sleep(1)
                self.close_gripper(collision_object, get_fingertip(self.transform_to(grasp)))

                # com = self.get_center_of_mass(collision_object)
                # com = self.tf.transform_to(com, "/tcp")
                # if com is None:
                # rospy.logwarn("TF failed")
                #     return False
                # self.load_object(self.calc_object_weight(collision_object, object_density),
                #                  Vector3(com.point.x, com.point.y, com.point.z))

                rospy.loginfo("grasped " + collision_object_name)

                # self.__grasp = self.tf.transform_to(grasp)
                # v1 = deepcopy(self.__grasp.pose.position)
                # v1.z = 0
                # v2 = deepcopy(collision_object.primitive_poses[0].position)
                # v2.z = 0
                # a = magnitude(subtract_point(v1, v2))
                # b = abs(self.__grasp.pose.position.z - collision_object.primitive_poses[0].position.z)
                # c = sqrt(a ** 2 + b ** 2)
                # self.__d = abs(c)
                # print c

                rospy.logdebug("lift object")
                if not self.__move_group_to(get_pre_grasp(grasp), move_group, blow_up=("map", collision_object_name)):
                    rospy.logdebug("couldnt lift object")
                return True
        rospy.logwarn("Grapsing failed.")
        return False

    def cmp_pose_stamped(self, collision_object, pose1, pose2):
        """
        Compares tow poses by calculating the distance to the centre of a collision object and
        returns -1/0/1 depending on which on is closer.
        :param collision_object: collision object as CollisionObject
        :param pose1: first pose as PoseStamped
        :param pose2: second pose as PoseStamped
        :return: pose1 > pose2
        """
        center = self.get_center_of_mass(collision_object)
        odom_pose1 = self.tf.transform_to(pose1)
        p1 = get_fingertip(odom_pose1)
        odom_pose2 = self.tf.transform_to(pose2)
        p2 = get_fingertip(odom_pose2)
        d1 = euclidean_distance(center.point, p1.point)
        d2 = euclidean_distance(center.point, p2.point)
        diff = d1 - d2
        if 0.0 <= abs(diff) <= 0.015 or len(collision_object.primitives) == 1:
            z1 = odom_pose1.pose.position.z
            z2 = odom_pose2.pose.position.z
            diff = z2 - z1
        return 1 if diff > 0 else -1 if diff < 0 else 0

    def filter_low_poses(self, list_of_poses):
        """
        Filters out positions that are very close to the ground.
        :param list_of_poses: list of PoseStamped in odom_combined
        :return: filtered list of PoseStamped
        """
        return [pose for pose in list_of_poses if self.tf.transform_to(pose).pose.position.z > min_grasp_height]

    def filter_close_poses(self, list_of_poses):
        base = self.get_base_origin()
        return [pose for pose in list_of_poses if
                euclidean_distance_in_2d(base.point, self.tf.transform_to(pose).pose.position) > 0.35]

    def calc_object_weight(self, collision_object, density):
        """
        Calculates the weight of a collision object with the given density
        :param collision_object: CollisionObject
        :param density: density as float
        :return: weight as float
        """
        return calc_object_volume(collision_object) * density

    def get_center_of_mass(self, collision_object):
        """
        Calculates the centre of a collision object.
        :param collision_object: CollisionObject
        :return: centre of mass as PointStamped
        """
        p = PointStamped()
        p.header.frame_id = "/odom_combined"
        for pose in collision_object.primitive_poses:
            p.point = add_point(p.point, pose.position)
        p.point = multiply_point(1.0 / len(collision_object.primitive_poses), p.point)

        return p

    def place(self, destination):
        """
        Deprecated. For testing only
        """
        return self.__place_with_group(destination, self.__arm_group)

    def place_and_move(self, destination):
        """
        Deprecated. For testing only
        """
        return self.__place_with_group(destination, self.__arm_base_group)

    def __place_with_group(self, destination, move_group):
        """
        Deprecated. For testing only
        """
        dest = deepcopy(destination)
        # print dest
        co = self.__planning_scene_interface.get_attached_object()
        if co is None:
            return False
        else:
            co = co.object
        dest = self.tf.transform_to(dest)
        place_poses = get_place_position(co, dest, self.tf.transform_to, self.__d, self.__grasp)
        # visualize_poses(place_poses)
        for place_pose in place_poses:
            if not self.__move_group_to(get_pre_place_position(place_pose), move_group):
                rospy.logwarn("Can't reach preplaceposition.")
                continue
            if not self.__move_group_to(place_pose, move_group):
                rospy.logwarn("Can't reach placeposition.")
                continue

            rospy.sleep(1)
            if not self.open_gripper():
                return False
            rospy.sleep(1)

            post_place_pose = self.tf.transform_to(place_pose, co.id)
            # post_place_pose.header.frame_id = "/tcp"
            # post_place_pose.pose.position = Point(0, 0, -post_place_length)

            if not self.__move_group_to(get_pre_grasp(post_place_pose), move_group):
                rospy.logwarn("Can't reach postplaceposition.")
                return True
            rospy.sleep(0.25)
            rospy.loginfo("placed " + co.id)
            return True
        return False

    def load_object(self, mass, cog):
        """
        Tells euroc that and object has been grasped.
        :param mass: mass of the object as float
        :param cog: centre of mass as Vector3
        :return: response message
        """
        request = SetObjectLoadRequest()
        request.mass = mass
        request.center_of_gravity = cog
        resp = self.__set_object_load_srv(request)
        # print resp.error_message
        return resp

    def get_planning_scene(self):
        return self.__planning_scene_interface

    def turn_arm(self, joint_value, joint=0):
        """
        Sets "link1" to "joint_value
        :param joint_value: float #radian -2.96 to 2.96
        :return: success of the movement
        """
        current_joint_values = self.__arm_group.get_current_joint_values()
        current_joint_values[joint] = joint_value
        self.__arm_group.set_joint_value_target(current_joint_values)
        path = self.__arm_group.plan()
        return self.__manService.move(path)

    def get_arm_move_group(self):
        return self.__arm_group

    def get_arm_base_move_group(self):
        return self.__arm_base_group

    # def set_height_constraint(self, t=True):
    # if t:
    # self.__planning_scene_interface.add_ground(0.95)
    #     else:
    #         self.__planning_scene_interface.remove_object("ground0.95")

    def pan_tilt(self, pan, tilt):
        """
        Moves the scene cam.
        :param pan: desired pan as float
        :param tilt: desired tilt as float
        :return: success of the movement
        """
        return self.__manService.pan_tilt(pan, tilt)

    def set_planning_time_arm(self, time):
        return self.__arm_group.set_planning_time(time)

    def direct_move(self, configuration):
        return self.__manService.direct_move(configuration)

    def scan_conveyor_pose(self):
        # Initialize DropPoint
        dp = geometry_msgs.msg.PoseStamped()
        dp.pose.position.x = 0
        dp.pose.position.y = 0
        dp.pose.position.z = -0.4
        dp.header.frame_id = "/drop_point"
        dp.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)

        rospy.logdebug('ScanConveyorPose: Transform DropPoint to odom')
        dp_odom = self.transform_to(dp)

        scan_conveyor_pose = geometry_msgs.msg.PoseStamped()
        scan_conveyor_pose.header.frame_id = "/mdl_middle"
        scan_conveyor_pose.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)

        scan_conveyor_pose.pose.position.x = 0
        scan_conveyor_pose.pose.position.y = -0.2
        scan_conveyor_pose.pose.position.z = 0

        rospy.logdebug('ScanConveyorPose: Transform mdl_middle to odom')
        mdl_middle_odom = self.transform_to(scan_conveyor_pose)

        scan_conveyor_pose.pose.position.z = mdl_middle_odom.pose.position.z + 0.3

        rospy.logdebug('ScanConveyorPose: Transform scan_conveyor_pose to odom')
        scan_conveyor_pose = self.transform_to(scan_conveyor_pose)

        rospy.logdebug('ScanConveyorPose: Calculate quaternion')
        quaternion = three_points_to_quaternion(scan_conveyor_pose.pose.position, dp_odom.pose.position)

        scan_conveyor_pose.pose.orientation = quaternion

        rospy.logdebug(scan_conveyor_pose)

        return scan_conveyor_pose

    def plan_to(self, pose):
        rospy.logdebug('PlanTo: Start planning ik')
        rospy.logdebug('PlanTo: ' + str(pose))
        service = rospy.ServiceProxy("/euroc_interface_node/search_ik_solution", SearchIkSolution)
        config = Configuration()
        list = self.get_current_lwr_joint_state()
        for i in range(len(list)):
            config.q.append(list[i])
        resp = service(config, pose)
        if resp.error_message:
            raise PlanningException(resp.error_message)
        rospy.logdebug('PlanTo: Return ik')
        return resp.solution

    def is_gripper_open(self):
        states = self.__gripper_group.get_current_joint_values()
        print "Finger: " + str(states)
        if states[0] < -0.03:
            return True
        # print states
        return False


# class for our own exception
class PlanningException(Exception):
    pass
