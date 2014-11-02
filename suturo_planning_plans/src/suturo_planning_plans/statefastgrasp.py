import copy
from math import pi
import numpy
from geometry_msgs.msg import Quaternion
import smach
import rospy
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
        # rospy.logdebug("FastGrasp: objects")
        # rospy.logdebug(resp.objects[0])
        self.__perceived_pose_time = resp.stamp
        # wait some time until the second perception
        rospy.sleep(Duration.from_sec(0.5))
        resp = service("cuboid")
        if len(resp.objects) == 0 or not resp.objects[0].c_cuboid_success:
            rospy.logdebug('FastGrasp: objects empty or no cuboid 2')
            return -2
        object2 = resp.objects[0].object
        time2 = resp.stamp
        # rospy.logdebug(resp.objects[0])
        # transform the points into odom_combined
        self.__perceived_pose = utils.manipulation.transform_to(object1)
        pose2 = utils.manipulation.transform_to(object2)
        # rospy.logdebug("FastGrasp: poses")
        # rospy.logdebug(__perceived_pose)
        # rospy.logdebug(pose2)
        # calculate the vector from the points of first and second perception
        self.__direction = numpy.array([(pose2.primitive_poses[0].position.x - self.__perceived_pose.primitive_poses[0].position.x),
                               (pose2.primitive_poses[0].position.y - self.__perceived_pose.primitive_poses[0].position.y)])
        # else:
        #     rospy.logerr("FastGrasp: __perceived_pose or pose2 empty!!!")
        # dir_dist = sqrt(pow(dir[0], 2) + pow(dir[1], 2))
        # get the duration between the two looks
        self.__time_between_poses = time2 - self.__perceived_pose_time

        rospy.logdebug('FastGrasp: Return Object')

        return 0

    def extrapolate(self, step):
        # set the time in n secs
        t = 5
        t_5 = rospy.Time.now() + rospy.Duration(t)
        time_13 = t_5 - self.__perceived_pose_time
        diff_time = float(time_13.to_sec()) / float(self.__time_between_poses.to_sec())
        dir_13 = numpy.array([diff_time * self.__direction[0], diff_time * self.__direction[1]])
        # add this new vector on the points from the first perception
        self.__pose_comp = copy.deepcopy(self.__perceived_pose)
        self.__pose_comp.primitive_poses[0].position.x += dir_13[0]
        self.__pose_comp.primitive_poses[0].position.y += dir_13[1]
        self.__pose_comp.id = "red_cube"

    # TODO: Abfangen, falls kein Plan zum Hochfahren des Armes, dann erst zurueck und dann hoch
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
        # extract the point from the object
        # t_point.orientation = quat
        # TODO: Was passiert wen kein Plan gefunden werden kann?
        rospy.logdebug('FastGrasp: Begin movement, Plan 1')
        for j in range(0, 4):
            self.extrapolate(j)
            self.calculate_target_point(self.__pose_comp)
            if j <= 2:
                try:
                    utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
                    break
                except:
                    rospy.logdebug("FastGrasp: Plan 1: No Plan fount in step " + str(j))
            else:
                return 'noPlanFound'

        # TODO: Timing zum Zupacken bestimmen!!!
        self.__t_point.position.z -= 0.07
        rospy.logdebug("FastGrasp: Plan 2")
        try:
            utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
        except:
            rospy.logdebug("FastGrasp: Plan 2: Cant grasp")
            return 'noPlanFound'

        utils.manipulation.close_gripper(self.__pose_comp)
        self.__t_point.position.z += 0.1
        rospy.logdebug("FastGrasp: Plan 3")
        # TODO: Bei Exception irgendwie ne andere Pose finden
        try:
            utils.manipulation.direct_move(utils.manipulation.plan_to(self.__t_point))
        except:
            rospy.logdebug("FastGrasp: Plan 3: Cant lift")
            return 'noPlanFound'

        rospy.sleep(Duration.from_sec(0.5))
        tfs = TorqueForceService()
        if tfs.is_free():
            rospy.logdebug("FastGrasp: Grasp Fail")
            return 'fail'
        rospy.logdebug("FastGrasp: objectGrasped, finished")
        return 'objectGrasped'