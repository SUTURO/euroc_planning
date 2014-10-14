import rospy
import scipy
import matplotlib.path as mplPath
import tf
from suturo_planning_visualization import visualization
from searchgrid import SearchGrid
from geometry import *
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
from geometry_msgs.msg import PoseStamped, PointStamped, Point


class ObjectFinder:
    _grid = None
    _collision_objects = []
    _fov_h = 0
    _fov_v = 0
    _fov_vectors = []
    _tf_listener = None

    def __init__(self, yaml):
        self._tf_listener = tf.TransformListener()
        self._grid = SearchGrid(10, 10, 2.0, 2.0)
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

        # Update visible fields
        visible_fields = self._get_visible_fields(intersection_points)
        for pos in visible_fields:
            self._grid.field[pos[0], pos[1]] += 1
        rospy.loginfo('Visible fields: %s' % str(visible_fields))

        # Visualize search grid
        visualization.publish_marker_array(self._grid.to_marker_array())

        # Visualize fov
        visualization.publish_lines(camera_pose.pose.position,
                                    map(lambda point_stamped: point_stamped.point, fov_points))

        # Visualize intersection fov lines with table
        map(lambda vec, name: visualization.publish_vector(vec, name), intersection_points, [5000, 6000, 7000, 8000])

    def get_place_to_search(self):
        focused_field = self._get_focused_field()
        x_s = focused_field[0]
        y_s = focused_field[1]
        lowest_value = 100
        best_field = None

        for i in range(1, len(self._grid.field)):
            for x_diff in range(-i, i):
                y_diff = i - abs(x_diff)
                x1 = x_s + x_diff
                y1 = y_s + y_diff
                x2 = x_s - x_diff
                y2 = y_s - y_diff
                if x1 < len(self._grid.field) and y1 < len(self._grid.field[0]) and \
                   self._grid.field[x1][y1] < lowest_value:
                    best_field = [x1, y1]
                    lowest_value = self._grid.field[x1][y1]
                if x2 < len(self._grid.field) and y2 < len(self._grid.field[0]) and \
                   self._grid.field[x2][y2] < lowest_value:
                    best_field = [x2, y2]
                    lowest_value = self._grid.field[x2][y2]
                if lowest_value == 0:
                    break
            # ends both loops if inner loop breaks, kinda hacky but should work
            else:
                continue
            break

        rospy.logdebug('Focused field: %s Field to search: %s' % (str(focused_field), str(best_field)))

        place_to_search = PointStamped()
        place_to_search.point = Point(best_field[0], best_field[1], 0)
        place_to_search.header.frame_id = '/odom_combined'
        rospy.logdebug('Place to search %s' % str(place_to_search))

        visualization.publish_vector(self._grid.coordinates[best_field[0], best_field[1]], 9999)

        return place_to_search

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

    def _get_visible_fields(self, intersection_points):
        points = map(lambda point: [point[0], point[1]], intersection_points)
        polygon = scipy.array([points[0], points[1], points[3], points[2]])

        return self._get_fields_in_polygon(polygon)

    def _get_fields_in_polygon(self, polygon):
        fields = []
        for x in range(0, len(self._grid.field)):
            for y in range(0, len(self._grid.field[0])):
                if self._in_polygon(polygon, (self._grid.coordinates[x][y][0], self._grid.coordinates[x][y][1])):
                    fields.append([x, y])
        return fields

    @staticmethod
    def _in_polygon(polygon, point):
        path = mplPath.Path(polygon)
        return path.contains_point(point)

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

    def _get_focused_field(self):
        camera_pose = self._get_camera_pose()
        x = camera_pose.pose.position.x
        y = camera_pose.pose.position.y
        offset_x = self._grid.field_size_x / 2.0
        offset_y = self._grid.field_size_y / 2.0
        polygon = scipy.array([[x - offset_x, y - offset_y], [x - offset_x, y + offset_y],
                               [x + offset_x, y + offset_y], [x + offset_x, y - offset_y]])

        fields = self._get_fields_in_polygon(polygon)
        if fields:
            return fields[0]
        else:
            return None
