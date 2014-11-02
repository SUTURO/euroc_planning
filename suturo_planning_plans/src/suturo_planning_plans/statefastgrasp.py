import copy
from math import pi, sqrt
import numpy
from geometry_msgs.msg import Quaternion
import smach
import rospy
from suturo_planning_manipulation.manipulation import PlanningException
from suturo_planning_manipulation.mathemagie import rotate_quaternion
from suturo_planning_manipulation.torque_force_service import TorqueForceService
import utils
from rospy.rostime import Duration
from suturo_perception_msgs.srv import GetGripper, geometry_msgs
from euroc_c2_msgs.srv import RequestNextObject


class FastGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectGrasped', 'timeExpired', 'noPlanFound', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])
        self.__perceived_pose = 0
        self.__perceived_pose_time = rospy.Time()
        self.__time_between_poses = rospy.Duration()
        self.__direction = 0
        self.__pose_comp = 0
        self.__t_point = geometry_msgs.msg.Pose()
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
        pose2 = utils.manipulation.transform_to(object2)

        # calculate the vector from the points of first and second perception
        self.__direction = numpy.array([(pose2.primitive_poses[0].position.x - self.__perceived_pose.primitive_poses[0].position.x),
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
            t = 2
        else:
            t = 1
        self.__t_point_time = rospy.Time.now() + rospy.Duration(7) + rospy.Duration(t * step)
        time_13 = self.__t_point_time - self.__perceived_pose_time
        diff_time = float(time_13.to_sec()) / float(self.__time_between_poses.to_sec())
        dir_13 = numpy.array([diff_time * self.__direction[0], diff_time * self.__direction[1]])
        # add this new vector on the points from the first perception
        self.__pose_comp = copy.deepcopy(self.__perceived_pose)
        self.__pose_comp.primitive_poses[0].position.x += dir_13[0]
        self.__pose_comp.primitive_poses[0].position.y += dir_13[1]
        self.__pose_comp.id = "red_cube"

    def calculate_target_point(self, pose_comp):
        self.__t_point = geometry_msgs.msg.Pose()
        self.__t_point.position = pose_comp.primitive_poses[0].position
        self.__t_point.position.z += 0.3
        quat = Quaternion()
        quat.x = pose_comp.primitive_poses[0].orientation.x
        quat.y = pose_comp.primitive_poses[0].orientation.y
        quat.z = pose_comp.primitive_poses[0].orientation.z
        quat.w = pose_comp.primitive_poses[0].orientation.w
        self.__t_point.orientation = rotate_quaternion(quat, -pi / 2, 0, 0)

    def execute(self, userdata):
        rospy.logdebug('FastGrasp: Executing state FastGrasp')
        # TODO: Nach ?? Sekunden time expired werfen

        if not utils.manipulation.is_gripper_open():
            rospy.logdebug('FastGrasp: Open Gripper')
            utils.manipulation.open_gripper()
        print utils.manipulation.is_gripper_open()

        r = self.percieve_object()
        i = 0
        serviceCall = False

        while r == -1:
            if i == 2:
                # TODO: Zeit anpassen, ab wann gecalled wird
                rospy.logdebug('FastGrasp: Request next object')
                rospy.ServiceProxy("/euroc_interface_node/request_next_object", RequestNextObject).call()
                serviceCall = True
            if i == 9 and serviceCall:
                rospy.logdebug('FastGrasp: Time Expired')
                return 'timeExpired'
            r = self.percieve_object()
            i += 1
        # TODO: Was passiert wenn das Objekt nur einmal gesehen wurde

        rospy.logdebug('FastGrasp: Begin movement, Plan 1')
        for j in range(0, 4):
            self.extrapolate(j)
            self.calculate_target_point(self.__pose_comp)
            if j <= 2:
                try:
                    utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
                    break
                except PlanningException:
                    rospy.logdebug("FastGrasp: Plan 1: No Plan fount in step " + str(j))
            else:
                return 'noPlanFound'

        while rospy.Time.now() < self.__t_point_time - rospy.Duration(0.5):
            rospy.sleep(0.01)
        # TODO: Timing zum Zupacken bestimmen!!!
        self.__t_point.position.z -= 0.07
        rospy.logdebug("FastGrasp: Plan 2")
        try:
            utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
        except PlanningException:
            rospy.logdebug("FastGrasp: Plan 2: Cant grasp")
            return 'noPlanFound'

        utils.manipulation.close_gripper(self.__pose_comp)
        self.__t_point.position.z += 0.1
        rospy.logdebug("FastGrasp: Plan 3")
        # TODO: Bei Exception irgendwie ne andere Pose finden
        try:
            utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
        except PlanningException:
            rospy.logdebug("FastGrasp: Plan 3: Cant lift")
            return 'noPlanFound'

        rospy.sleep(Duration.from_sec(0.5))
        tfs = TorqueForceService()
        if tfs.is_free():
            rospy.logdebug("FastGrasp: Grasp Fail")
            return 'fail'
        rospy.logdebug("FastGrasp: objectGrasped, finished")
        return 'objectGrasped'