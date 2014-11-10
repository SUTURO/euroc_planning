import copy
from math import pi, sqrt
import numpy
from geometry_msgs.msg import Quaternion
from rospy.service import ServiceException
import smach
import rospy
from suturo_planning_manipulation.manipulation import PlanningException
from suturo_planning_manipulation.mathemagie import rotate_quaternion, get_yaw, euler_to_quaternion
from suturo_planning_manipulation.torque_force_service import TorqueForceService
import utils
from rospy.rostime import Duration
from suturo_perception_msgs.srv import GetGripper, geometry_msgs
from euroc_c2_msgs.srv import RequestNextObject


class FastGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectGrasped', 'timeExpired', 'noPlanFound', 'graspingFailed',
                                             'noObjectsLeft', 'fail'],
                             input_keys=['yaml', 'request_second_object', 'object_index'],
                             output_keys=['request_second_object', 'object_index'])
        self.__perceived_pose = 0
        self.__perceived_pose_time = rospy.Time()
        self.__time_between_poses = rospy.Duration()
        self.__direction = 0
        self.__pose_comp = 0
        self.__t_point = geometry_msgs.msg.PoseStamped()
        self.__t_point_time = 0

    def percieve_object(self):
        rospy.logdebug('FastGrasp: Start percieving Object')
        # create service
        service = rospy.ServiceProxy("/suturo/GetGripper", GetGripper)

        # get the first perception
        rospy.logdebug('FastGrasp: call Perception Service')
        resp = service("firstConveyorCall,cuboid")
        if len(resp.objects) == 0 or not resp.objects[0].c_cuboid_success:
            rospy.logdebug('FastGrasp: objects empty or no cuboid 1')
            return -1
        object1 = resp.objects[0].object
        self.__perceived_pose_time = resp.stamp

        # wait some time until the second perception
        rospy.sleep(Duration.from_sec(0.5))
        resp = service("cuboid")
        if len(resp.objects) == 0 or not resp.objects[0].c_cuboid_success:
            rospy.logdebug('FastGrasp: objects empty or no cuboid 2')
            return -2
        object2 = resp.objects[0].object
        time2 = resp.stamp

        # transform the points into odom_combined
        self.__perceived_pose = utils.manipulation.transform_to(object1)
        # utils.manipulation.get_planning_scene().add_object(self.__perceived_pose)
        pose2 = utils.manipulation.transform_to(object2)

        # calculate the vector from the points of first and second perception
        self.__direction = numpy.array(
            [(pose2.primitive_poses[0].position.x - self.__perceived_pose.primitive_poses[0].position.x),
             (pose2.primitive_poses[0].position.y - self.__perceived_pose.primitive_poses[0].position.y)])

        # calculate the time between the first and second perception
        self.__time_between_poses = time2 - self.__perceived_pose_time

        rospy.logdebug('FastGrasp: Return Object')

        return 0

    def extrapolate(self, step):
        dir_dist = sqrt(pow(self.__direction[0], 2) + pow(self.__direction[1], 2))
        if dir_dist < 0.03:
            t = 10
        elif dir_dist < 0.2:
            t = 3
        else:
            t = 1
        self.__t_point_time = rospy.Time.now() + rospy.Duration(10) + rospy.Duration(t * step)
        time_13 = self.__t_point_time - self.__perceived_pose_time
        diff_time = float(time_13.to_sec()) / float(self.__time_between_poses.to_sec())
        dir_13 = numpy.array([diff_time * self.__direction[0], diff_time * self.__direction[1]])
        # add this new vector on the points from the first perception
        self.__pose_comp = copy.deepcopy(self.__perceived_pose)
        self.__pose_comp.primitive_poses[0].position.x += dir_13[0]
        self.__pose_comp.primitive_poses[0].position.y += dir_13[1]
        self.__pose_comp.id = "red_cube"

    def calculate_target_point(self, pose_comp):
        self.__t_point.pose.position = pose_comp.primitive_poses[0].position
        self.__t_point.pose.position.z += 0.3
        quat = Quaternion()
        quat.x = pose_comp.primitive_poses[0].orientation.x
        quat.y = pose_comp.primitive_poses[0].orientation.y
        quat.z = pose_comp.primitive_poses[0].orientation.z
        quat.w = pose_comp.primitive_poses[0].orientation.w
        print get_yaw(quat)
        self.__t_point.pose.orientation = euler_to_quaternion(0, pi / 2, get_yaw(quat) + (pi / 2))

    def execute(self, userdata):
        rospy.logdebug('FastGrasp: Executing state FastGrasp')
        self.__t_point.header.frame_id = "/odom_combined"
        # TODO: Auf 10 Min grenzen testen

        if not utils.manipulation.is_gripper_open():
            rospy.logdebug('FastGrasp: Open Gripper')
            utils.manipulation.open_gripper()

        r = self.percieve_object()
        i = 0

        while r == -1:
            if i == 2 and not userdata.request_second_object:
                # TODO: Zeit anpassen, ab wann gecalled wird
                rospy.logdebug('FastGrasp: Request next object')
                rospy.ServiceProxy("/euroc_interface_node/request_next_object", RequestNextObject).call()
                userdata.request_second_object = True
            if i == 19 and userdata.request_second_object:
                rospy.logdebug('FastGrasp: Time Expired')
                return 'noObjectsLeft'
            r = self.percieve_object()
            i += 1
        # TODO: Was passiert wenn das Objekt nur einmal gesehen wurde

        rospy.logdebug('FastGrasp: Begin movement, Plan 1')
        for j in range(0, 4):
            self.extrapolate(j)
            self.calculate_target_point(self.__pose_comp)
            if utils.manipulation.move_to(self.__t_point):
                rospy.logdebug("FastGrasp: Plan 1: moved!")
                break
            else:
                rospy.logdebug("FastGrasp: Plan 1: No Plan fount in step " + str(j))
                if j == 3:
                    return 'noPlanFound'
        if userdata.object_index == 1:
            offset = rospy.Duration(1.5)
        elif userdata.object_index == 2:
            offset = rospy.Duration(1)
        else:
            offset = rospy.Duration(2)
        while rospy.Time.now() < self.__t_point_time - offset:
            rospy.sleep(0.01)
        self.__t_point.pose.position.z -= 0.07
        rospy.logdebug("FastGrasp: Plan 2")
        if utils.manipulation.move_to(self.__t_point):
            rospy.logdebug("FastGrasp: Plan 2: moved!")
        else:
            rospy.logdebug("FastGrasp: Plan 2: Cant grasp")
            return 'noPlanFound'

        utils.manipulation.close_gripper(self.__pose_comp)

        self.__t_point.pose.position.z += 0.1
        rospy.logdebug("FastGrasp: Plan 3")
        for k in range(0, 4):
            if utils.manipulation.move_to(self.__t_point):
                rospy.logdebug("FastGrasp: Plan 3: moved!")
                break
            else:
                if k == 3:
                    rospy.logdebug("FastGrasp: No Plan found to lift object")
                    return 'noPlanFound'
                rospy.logdebug("FastGrasp: Plan 3: Cant lift: " + str(k))
                if self.__t_point.pose.position.y > 0:
                    self.__t_point.pose.position.y -= 0.1
                else:
                    self.__t_point.pose.position.y += 0.1
                if self.__t_point.pose.position.x > 0:
                    self.__t_point.pose.position.x -= 0.1
                else:
                    self.__t_point.pose.position.x += 0.1

        rospy.sleep(Duration.from_sec(0.5))
        tfs = TorqueForceService()
        if tfs.is_free():
            rospy.logdebug("FastGrasp: Grasp Fail")
            return 'graspingFailed'
        rospy.logdebug("FastGrasp: objectGrasped, finished")
        userdata.object_index += 1
        return 'objectGrasped'