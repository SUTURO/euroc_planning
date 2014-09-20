import rospy
from searchgrid import SearchGrid
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


class ObjectFinder:

    _grid = None
    _collision_objects = []

    def __init__(self, yaml):
        self._grid = SearchGrid(50, 50)

    def update_search_grid(self):
        # TODO Get planning scene from moveit service
        # TODO Get gripper pose and calculate visible fields
        None

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
