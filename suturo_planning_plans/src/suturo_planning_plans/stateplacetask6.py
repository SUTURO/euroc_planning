__author__ = 'thocar'

import smach
import rospy
import geometry_msgs
from geometry_msgs.msg import Quaternion
import utils
from suturo_planning_manipulation.manipulation import Manipulation, PlanningException
from tf.listener import TransformListener
from math import pi
from suturo_planning_manipulation.mathemagie import rotate_quaternion
import random
import time


class PlaceTask6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

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

        place_pose = self.get_place_point(userdata)

        rospy.logdebug('PlaceTask6: Begin to plan...')
        # TODO: Abfangen, wenn kein Plan gefunden wird und die rad_fac erhoehen.
        place_pose_plan = self.plan_place_point(place_pose, userdata)

        rospy.logdebug('PlaceTask6: Move Arm to Place Pose')
        rospy.logdebug(place_pose)
        utils.manipulation.direct_move(place_pose_plan)
        rospy.logdebug('PlaceTask6: OpenGripper')
        utils.manipulation.open_gripper()

        return 'success'

    def plan_place_point(self, place_pose, userdata):
        for i in range(0, 3):
            try:
                plan = utils.manipulation.plan_to(place_pose)
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

        target_zone_odom = utils.manipulation.transform_to(target_zone_pose)

        place_pose = geometry_msgs.msg.Pose()
        place_pose.position = target_zone_odom.pose.position
        place_pose.position.z += 0.5
        place_pose.orientation = rotate_quaternion(target_zone_odom.pose.orientation, -pi, 0, 0)
        # place_pose.orientation = target_zone_odom.pose.orientation

        return place_pose