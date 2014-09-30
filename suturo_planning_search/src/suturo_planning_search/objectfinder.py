import rospy
import scipy
import tf
import rospy
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
    _fov_vectors = []
    _tf_listener = None

    def __init__(self, yaml):
        self._tf_listener = tf.TransformListener()
        self._grid = SearchGrid(10, 10)
        camera = yaml.sensors[0].camera
        self._fov_h = camera.horizontal_fov
        self._fov_v = 2.0 * scipy.arctan(scipy.tan(self._fov_h / 2.0) * (camera.image_height / camera.image_width))
        self._fov_vectors = fov_vectors(self._fov_h, self._fov_v)

    def update_search_grid(self):
        # TODO Get planning scene from moveit service
        # TODO Get gripper pose and calculate visible fields

        # Create a PoseStamped for the camera
        camera_pose = self._get_camera_pose()
        rospy.logdebug('Camera pose: %s' % str(camera_pose))

        # Get the points for the fov
        fov_points = self._get_fov_points()
        rospy.logdebug('FOV points: %s' % str(fov_points))

        # Get the intersection points of the fov lines with the table
        table_points = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]]
        point_to_vector = lambda point: scipy.array([point.x, point.y, point.z])
        intersection_points = map(lambda fov_point: intersection_line_plane(point_to_vector(camera_pose.pose.position),
                                                                            point_to_vector(fov_point.point),
                                                                            *table_points),
                                  fov_points)
        rospy.logdebug('Intersection points fov lines with table: %s' % str(intersection_points))

        # Visualize search grid
        visualization.publish_marker_array(self._grid.to_marker_array())

        # Visualize fov
        visualization.publish_lines(camera_pose.pose.position,
                                    map(lambda point_stamped: point_stamped.point, fov_points))

        # Visualize intersection fov lines with table
        map(lambda vec, name: visualization.publish_vector(vec, name), intersection_points, [5000, 6000, 7000, 8000])

    def get_place_to_search(self):
        self

    def _get_camera_pose(self):
        camera_pose_tdepth = PoseStamped()
        camera_pose_tdepth.header.stamp = rospy.Time(0)
        camera_pose_tdepth.header.frame_id = '/tdepth'
        camera_pose_tdepth.pose.orientation.w = 1

        camera_pose = None
        try:
            camera_pose = self._tf_listener.transformPose('/odom_combined', camera_pose_tdepth)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'camera tf lookup failed'

        return camera_pose

    def _get_fov_points(self):
        fov_points = []

        for vector in self._fov_vectors:
            point = PointStamped()
            point.header.stamp = rospy.Time(0)
            point.header.frame_id = '/tdepth'
            point.point.x = vector[0]
            point.point.y = vector[1]
            point.point.z = vector[2]

            try:
                fov_points.append(self._tf_listener.transformPoint('/odom_combined', point))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print 'fov points tf lookup failed'
                continue

        return fov_points

    def _update_collision_objects(self):
        self

    def _is_visible(self):
        self

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
