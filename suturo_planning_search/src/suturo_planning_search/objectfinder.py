import rospy
import math
import tf
from suturo_planning_visualization import visualization
from searchgrid import SearchGrid
from geometry import *
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from geometry_msgs.msg import PoseStamped, PointStamped


class ObjectFinder:

    _grid = None
    _collision_objects = []
    _fov_h = 0
    _fov_v = 0
    _tf_listener = None

    def __init__(self, yaml):
        self._tf_listener = tf.TransformListener()
        self._grid = SearchGrid(10, 10)
        camera = yaml.sensors[0].camera
        self._fov_h = camera.horizontal_fov
        self._fov_v = 2.0 * math.atan(math.tan(self._fov_h / 2.0) * (camera.image_width / camera.image_height))

    def update_search_grid(self):
        # TODO Get planning scene from moveit service
        # TODO Get gripper pose and calculate visible fields

        # Create a PoseStamped for the camera
        camera_pose_tdepth = PoseStamped()
        camera_pose_tdepth.header.stamp = rospy.get_rostime()
        camera_pose_tdepth.header.frame_id = '/tdepth'
        camera_pose_tdepth.pose.orientation.w = 1

        camera_pose = None
        try:
            camera_pose = self._tf_listener.transformPose('/odom_combined', camera_pose_tdepth)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            None

        fov_points = []

        for vector in fov_vectors(self._fov_h, self._fov_v):
            point = PointStamped()
            point.header.stamp = rospy.get_rostime()
            point.header.frame_id = '/tdepth'
            point.point.x = vector[0]
            point.point.y = vector[1]
            point.point.z = vector[2]

            try:
                fov_points.append(self._tf_listener.transformPoint('/odom_combined', point))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        visualization.publish_marker_array(self._grid.to_marker_array())

        visualization.publish_lines(camera_pose.pose.position,
                                    map(lambda pose_stamped: pose_stamped.pose.position, fov_points))

    def get_place_to_search(self):
        None

    def _update_collision_objects(self):
        None

    def _is_visible(self):
        None

    @staticmethod
    def _get_collision_objects():
        rospy.wait_for_service('get_planning_scene')
        try:
            scene = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
            comp = PlanningSceneComponents
            comp.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            return scene(comp).scene.world.collision_objects
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
