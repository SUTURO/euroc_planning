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
        smach.State.__init__(self, outcomes=['objectGrasped', 'timeExpired', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def percieve_object(self, t):
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
        time1 = resp.stamp
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
        pose1 = utils.manipulation.transform_to(object1)
        pose2 = utils.manipulation.transform_to(object2)
        # rospy.logdebug("FastGrasp: poses")
        # rospy.logdebug(pose1)
        # rospy.logdebug(pose2)
        # calculate the vector from the points of first and second perception
        # if (pose2.primitive_poses.__len__() > 0) & (pose1.primitive_poses.__len__() > 0):
        # if pose2 is not None & pose1 is not None:
        dir = numpy.array([(pose2.primitive_poses[0].position.x - pose1.primitive_poses[0].position.x),
                               (pose2.primitive_poses[0].position.y - pose1.primitive_poses[0].position.y)])
        # else:
        #     rospy.logerr("FastGrasp: pose1 or pose2 empty!!!")
        # dir_dist = sqrt(pow(dir[0], 2) + pow(dir[1], 2))
        # get the duration between the two looks
        time_12 = time2 - time1
        # TODO: Extrapolation auslagern
        # set the time in n secs
        t_5 = rospy.Time.now() + rospy.Duration(t)
        time_13 = t_5 - time1
        diff_time = float(time_13.to_sec()) / float(time_12.to_sec())
        dir_13 = numpy.array([diff_time * dir[0], diff_time * dir[1]])
        # add this new vector on the points from the first perception
        pose_comp = copy.deepcopy(pose1)
        pose_comp.primitive_poses[0].position.x += dir_13[0]
        pose_comp.primitive_poses[0].position.y += dir_13[1]
        pose_comp.id = "red_cube"
        rospy.logdebug('FastGrasp: Return Object')
        return pose_comp

    # TODO: Abfangen, falls kein Plan zum Hochfahren des Armes, dann erst zurueck und dann hoch
    def execute(self, userdata):
        rospy.logdebug('FastGrasp: Executing state FastGrasp')
        # TODO: Nach ?? Sekunden time expired werfen
        if utils.manipulation.is_gripper_open():
            rospy.logdebug('FastGrasp: Open Gripper')
            utils.manipulation.open_gripper()

        # make sure that calculation succeeded
        pose_comp = self.percieve_object(10)
        i = 0
        while pose_comp == -1:
            if i == 2:
                # TODO: Zeit anpassen, ab wann gecalled wird
                rospy.logdebug('FastGrasp: Request next object')
                rospy.ServiceProxy("/euroc_interface_node/request_next_object", RequestNextObject).call()
            pose_comp = self.percieve_object(10)
            i += 1
        # TODO: Was passiert wenn das Objekt nur einmal gesehen wurde
        # extract the point from the object
        t_point = geometry_msgs.msg.Pose()
        t_point.position = pose_comp.primitive_poses[0].position
        t_point.position.z += 0.3
        quat = Quaternion()
        quat.x = pose_comp.primitive_poses[0].orientation.x
        quat.y = pose_comp.primitive_poses[0].orientation.y
        quat.z = pose_comp.primitive_poses[0].orientation.z
        quat.w = pose_comp.primitive_poses[0].orientation.w
        t_point.orientation = rotate_quaternion(quat, -pi / 2, 0, 0)
        # t_point.orientation = quat
        # TODO: Was passiert wen kein Plan gefunden werden kann?
        rospy.logdebug('FastGrasp: Begin movement, Plan 1')
        utils.manipulation.direct_move(utils.manipulation.plan_to(t_point))

        # TODO: Timing zum Zupacken bestimmen!!!
        t_point.position.z -= 0.07
        rospy.logdebug("FastGrasp: Plan 2")
        utils.manipulation.direct_move(utils.manipulation.plan_to(t_point))
        utils.manipulation.close_gripper(pose_comp)
        t_point.position.z += 0.1
        rospy.logdebug("FastGrasp: Plan 3")
        utils.manipulation.direct_move(utils.manipulation.plan_to(t_point))
        rospy.sleep(Duration.from_sec(0.5))
        tfs = TorqueForceService()
        if tfs.is_free():
            rospy.logdebug("FastGrasp: Grasp Fail")
            return 'fail'
        rospy.logdebug("FastGrasp: objectGrasped, finished")
        return 'objectGrasped'