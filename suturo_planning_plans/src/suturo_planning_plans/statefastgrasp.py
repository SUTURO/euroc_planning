import copy
from math import pi
import numpy
import euroc_c2_msgs
from euroc_c2_msgs.srv import SearchIkSolution
from euroc_c2_msgs.msg import Configuration
from geometry_msgs.msg import Quaternion
import smach
import rospy
from suturo_planning_manipulation.mathemagie import rotate_quaternion
from suturo_planning_manipulation.torque_force_service import TorqueForceService
import utils
from rospy.rostime import Duration

from suturo_perception_msgs.srv import GetGripper, geometry_msgs


class FastGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['objectGrasped', 'timeExpired', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def plan_to(self, pose):
        rospy.loginfo('FastGrasp: Start planning ik')
        service = rospy.ServiceProxy("/euroc_interface_node/search_ik_solution", SearchIkSolution)
        config = Configuration()
        list = utils.manipulation.get_current_lwr_joint_state()
        for i in range(len(list)):
                    config.q.append(list[i])
        resp = service(config, pose)
        if resp.error_message:
            raise Exception(resp.error_message)
        rospy.loginfo('FastGrasp: Return ik')
        return resp.solution

    def percieve_object(self, t):
        rospy.loginfo('FastGrasp: Start percieving Object')
        # create service
        service = rospy.ServiceProxy("/suturo/GetGripper", GetGripper)
        # get the first perception
        rospy.loginfo('FastGrasp: call Perception Service')
        resp = service("firstConveyorCall,centroid,cuboid")
        if len(resp.objects) == 0:
            return -1
        object1 = resp.objects[0].object
        time1 = resp.stamp
        # wait some time until the second perception
        rospy.sleep(Duration.from_sec(0.5))
        resp = service("centroid,cuboid")
        if len(resp.objects) == 0:
            return -2
        object2 = resp.objects[0].object
        time2 = resp.stamp
        # transform the points into odom_combined
        pose1 = utils.manipulation.transform_to(object1)
        pose2 = utils.manipulation.transform_to(object2)
        # calculate the vector from the points of first and second perception
        dir = numpy.array([(pose2.primitive_poses[0].position.x - pose1.primitive_poses[0].position.x), (
            pose2.primitive_poses[0].position.y - pose1.primitive_poses[0].position.y)])
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
        rospy.loginfo('FastGrasp: Return Object')
        return pose_comp

    def execute(self, userdata):
        rospy.loginfo('FastGrasp: Executing state FastGrasp')
        # TODO: Nach ?? Sekunden time expired werfen

        utils.manipulation.open_gripper()

        # make sure that calculation succeeded
        pose_comp = self.percieve_object(30)
        while pose_comp == -1:
            pose_comp = self.percieve_object(30)
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
        t_point.orientation = rotate_quaternion(quat, -pi/2, 0, 0)
        # t_point.orientation = quat
        # TODO: Was passiert wen kein Plan gefunden werden kann?
        rospy.loginfo('FastGrasp: Begin movement')
        utils.manipulation.direct_move(self.plan_to(t_point))

        # TODO: Timing zum Zupacken bestimmen!!!
        t_point.position.z -= 0.07
        utils.manipulation.direct_move(self.plan_to(t_point))
        utils.manipulation.close_gripper(pose_comp)
        t_point.position.z += 0.1
        utils.manipulation.direct_move(self.plan_to(t_point))
        rospy.sleep(Duration.from_sec(0.5))
        tfs = TorqueForceService()
        if tfs.is_free():
            return 'fail'
        return 'objectGrasped'