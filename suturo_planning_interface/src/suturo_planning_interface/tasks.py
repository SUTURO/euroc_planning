import rospy
from suturo_perception_msgs.msg import EurocObject
from suturo_interface_msgs.msg import TaskData
import suturo_msgs.msg
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from suturo_msgs.msg import Task

class TaskInit(object):
    def __init__(self):
        pass


class Task(object):
    def __init__(self):
        self.data = create_default_task_data()

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


def create_default_task_data():
    taskdata = TaskData()
    taskdata.objects_found = []
    taskdata.classified_objects = []
    taskdata.object_to_focus = EurocObject()
    taskdata.objects_to_focus = []
    #taskdata.perceived_objects = []
    taskdata.fitted_object = EurocObject()
    taskdata.fitted_objects = []
    taskdata.enable_movement = False
    taskdata.name = ""
    #taskdata.cell_coords = []
    taskdata.sec_try = False
    taskdata.sec_try_done = False
    taskdata.failed_object = EurocObject()
    taskdata.initialization_time = rospy.get_rostime()
    taskdata.logging = 0
    taskdata.yaml = suturo_msgs.msg.Task()
    taskdata.object_to_move = EurocObject()
    taskdata.focused_point = Point()
    taskdata.clean_up_plan = []
    taskdata.place_position = PointStamped()
    taskdata.dist_to_obj = 0
    taskdata.grasp = PoseStamped()
    taskdata.placed_object = ""
    return taskdata
