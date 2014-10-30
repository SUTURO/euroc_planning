__author__ = 'thocar'

import smach
import rospy
from euroc_c2_msgs.srv import SearchIkSolution
from euroc_c2_msgs.msg import Configuration
import geometry_msgs
from geometry_msgs.msg import Quaternion
import utils
from suturo_planning_manipulation.manipulation import Manipulation
from tf.listener import TransformListener
from math import pi
from suturo_planning_manipulation.mathemagie import rotate_quaternion

class PlaceTask6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'fail'],
                             input_keys=['yaml'],
                             output_keys=[])

    def plan_to(self, pose):
        rospy.logdebug('PlaceTask6: Start planning ik')
        service = rospy.ServiceProxy("/euroc_interface_node/search_ik_solution", SearchIkSolution)
        config = Configuration()
        list = utils.manipulation.get_current_lwr_joint_state()
        for i in range(len(list)):
                    config.q.append(list[i])
        resp = service(config, pose)
        if resp.error_message:
            raise Exception(resp.error_message)
        rospy.logdebug('PlaceTask6: Return ik')
        return resp.solution

    def execute(self, userdata):
        rospy.logdebug('PlaceTask6: Executing state PlaceTask6')

        listener = TransformListener()

        if utils.manipulation is None:
            utils.manipulation = Manipulation()
            rospy.sleep(2)

        # TODO: Irgend ne Abbruchbedingung machen (nach x Sekunden)
        while not listener.frameExists("target_zone"):
            rospy.loginfo("wait for target_zone frame")
            rospy.sleep(2.)

        target_zone_pose = geometry_msgs.msg.PoseStamped()
        target_zone_pose.pose.position.x = 0
        target_zone_pose.pose.position.y = 0
        target_zone_pose.pose.position.z = 0
        target_zone_pose.pose.orientation = geometry_msgs.msg.Quaternion(0.0, 0.0, 0.0, 1.0)
        target_zone_pose.header.frame_id = "/target_zone"

        target_zone_odom = utils.manipulation.transform_to(target_zone_pose)

        place_pose = geometry_msgs.msg.Pose()
        place_pose.position = target_zone_odom.pose.position
        place_pose.position.z += 0.2
        place_pose.orientation = rotate_quaternion(target_zone_odom.pose.orientation, -pi/2, 0, 0)
        # place_pose.orientation = target_zone_odom.pose.orientation

        rospy.logdebug('PlaceTask6: Move Arm to Place Pose')
        utils.manipulation.direct_move(self.plan_to(place_pose))
        rospy.logdebug('PlaceTask6: OpenGripper')
        utils.manipulation.open_gripper()

        return 'success'