from searchgrid import SearchGrid


class ObjectFinder:

    _grid = None
    _collision_objects = []

    def __init__(self, yaml):
        self._grid = SearchGrid(50, 50)

    def update_search_grid(self):
        None

    def get_place_to_search(self):
        None

    def _update_collision_objects(self):
        None

    def _get_visible_fields(self):
        None
