from copy import deepcopy
from geometry_msgs.msg._PointStamped import PointStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Vector3Stamped import Vector3Stamped
from moveit_msgs.msg._CollisionObject import CollisionObject
import rospy
from sensor_msgs.msg._PointCloud2 import PointCloud2
from tf.listener import TransformListener

__author__ = 'ichumuh'

class Transformer:

    def __init__(self):
        self.__listener = TransformListener()

    def __del__(self):
        pass

    def transform_to(self, pose_target, target_frame="/odom_combined"):
        '''
        Transforms the pose_target into the target_frame.
        :param pose_target: object to transform as PoseStamped/PointStamped/Vector3Stamped/CollisionObject/PointCloud2
        :param target_frame: goal frame id
        :return: transformed object
        '''
        odom_pose = None
        i = 0
        while odom_pose is None and i < 10:
            try:
                #now = rospy.Time.now()
                #self.__listener.waitForTransform(target_frame, pose_target.header.frame_id, now, rospy.Duration(4))
                if type(pose_target) is CollisionObject:
                    i = 0
                    new_co = deepcopy(pose_target)
                    for cop in pose_target.primitive_poses:
                        p = PoseStamped()
                        p.header = pose_target.header
                        p.pose.orientation = cop.orientation
                        p.pose.position = cop.position
                        p = self.transform_to(p, target_frame)
                        if p is None:
                            return None
                        new_co.primitive_poses[i].position = p.pose.position
                        new_co.primitive_poses[i].orientation = p.pose.orientation
                        i += 1
                    new_co.header.frame_id = target_frame
                    return new_co
                if type(pose_target) is PoseStamped:
                    odom_pose = self.__listener.transformPose(target_frame, pose_target)
                    break
                if type(pose_target) is Vector3Stamped:
                    odom_pose = self.__listener.transformVector3(target_frame, pose_target)
                    break
                if type(pose_target) is PointStamped:
                    odom_pose = self.__listener.transformPoint(target_frame, pose_target)
                    break
                if type(pose_target) is PointCloud2:
                    odom_pose = self.__listener.transformPointCloud(target_frame, pose_target)
                    break

            except Exception, e:
                print "tf error:::", e
            rospy.sleep(0.5)

            i += 1
            rospy.logdebug("tf fail nr. " + str(i))

        if odom_pose is None:
            rospy.logerr("FUUUUUUUUUUUUUU!!!! fucking tf shit!!!!")
        return odom_pose
