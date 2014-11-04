__author__ = 'thocar'

import smach
import rospy
import geometry_msgs
from geometry_msgs.msg import Quaternion
import utils
from suturo_planning_manipulation.manipulation import Manipulation, PlanningException
from tf.listener import TransformListener
from math import pi
from suturo_planning_manipulation.mathemagie import rotate_quaternion, euler_to_quaternion
import random
import time


class PlaceTask6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])
        self.__place_pose = 0

    def execute(self, userdata):
        rospy.logdebug('PlaceTask6: Executing state PlaceTask6')

        listener = TransformListener()

        if utils.manipulation is None:
            utils.manipulation = Manipulation()

        # TODO: Exception schmeissen wenn abort_after vergangen ist
        abort_after = 15
        then = int(time.time())
        now = int(time.time())
        while not now - then < abort_after and not listener.frameExists("target_zone"):
            rospy.loginfo("wait for target_zone frame")
            rospy.sleep(2.)
            now = int(time.time())

        self.__place_pose = self.get_place_point(userdata)

        rospy.logdebug('PlaceTask6: Move Arm to Place Pose')
        rospy.logdebug(self.__place_pose)
        for i in range(0, 7):
            if utils.manipulation.move_to(self.__place_pose, False):
                rospy.logdebug('PlaceTask6: OpenGripper')
                utils.manipulation.open_gripper()
                return 'success'
            else:
                rospy.logdebug('PlaceTask6: Calculate new place position...')
                if i <= 2:
                    self.__place_pose = self.get_place_point(userdata)
                if i > 2:
                    self.__place_pose = self.get_place_point(userdata, 3)
                if i == 6:
                    rospy.logdebug('PlaceTask6: Cant find place position!')
                    utils.manipulation.open_gripper()
                    return 'fail'

    def plan_place_point(self, place_pose, userdata):
        for i in range(0, 3):
            try:
                plan = utils.manipulation.plan(utils.manipulation.get_arm_move_group(), place_pose)
                rospy.logdebug('PlaceTask6: Plan found')
                return plan
            except PlanningException:
                rospy.logdebug('PlaceTask6: Plan exception!')
                place_pose = self.get_place_point(userdata)

        return "noPlanFound"

    def get_place_point(self, userdata, rad_fac=2):
        target_zone_pose = geometry_msgs.msg.PoseStamped()
        target_zone_pose.pose.position.x = random.uniform(-userdata.yaml.target_zones[0].max_distance / rad_fac,
                                                          userdata.yaml.target_zones[0].max_distance / rad_fac)
        target_zone_pose.pose.position.y = random.uniform(-userdata.yaml.target_zones[0].max_distance / rad_fac,
                                                          userdata.yaml.target_zones[0].max_distance / rad_fac)
        target_zone_pose.pose.position.z = 0
        target_zone_pose.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
        target_zone_pose.header.frame_id = "/target_zone"

        self.__place_pose = utils.manipulation.transform_to(target_zone_pose)

        self.__place_pose.pose.orientation = euler_to_quaternion(0, pi / 2, 0)

        self.__place_pose.pose.position.z += 0.5

        return self.__place_pose