import rospy


class TaskInit(object):
    def __init__(self):
        pass


class Task1(object):
    def __init__(self):
        self.data = TaskData(name="Task1", enable_movement=True)


# TODO: irgendwo anders hin packen
class TaskData(object):
    def __init__(self, name=None, enable_movement=False):
        self.objects_found = []
        self.perceived_objects = []
        self.fitted_object = None
        self.fitted_objects = []
        self.enable_movement = enable_movement
        self.name = name
        self.cell_coords = []
        self.sec_try = False
        self.sec_try_done = False
        self.failed_object = None
        self.initialization_time = None
        self.logging = None
        self.yaml = None
