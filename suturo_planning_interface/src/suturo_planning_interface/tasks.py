import rospy


class TaskInit(object):
    def __init__(self):
        pass


class Task(object):
    def __init__(self):
        self.data = TaskData()


class Task1(Task):
    def __init__(self):
        super(Task1, self).__init__()
        self.data.name = "task1"


class Task2(Task):
    def __init__(self):
        super(Task2, self).__init__()
        self.data.name = "task2"


class Task3(Task):
    def __init__(self):
        super(Task3, self).__init__()
        self.data.name = "task3"


class Task4(Task):
    def __init__(self):
        super(Task4, self).__init__()
        self.data.name = "task4"


class Task5(Task):
    def __init__(self):
        super(Task5, self).__init__()
        self.data.name = "task5"


class Task6(Task):
    def __init__(self):
        super(Task6, self).__init__()
        self.data.name = "task6"


# TODO: irgendwo anders hin packen
class TaskData(object):
    def __init__(self):
        self.objects_found = []
        self.perceived_objects = []
        self.fitted_object = None
        self.fitted_objects = []
        self.enable_movement = False
        self.name = None
        self.cell_coords = []
        self.sec_try = False
        self.sec_try_done = False
        self.failed_object = None
        self.initialization_time = None
        self.logging = None
        self.yaml = None
