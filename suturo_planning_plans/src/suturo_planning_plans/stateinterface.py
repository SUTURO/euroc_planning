from geometry_msgs.msg._Point import Point
import smach
import rospy
from geometry_msgs.msg import PoseStamped
from suturo_planning_manipulation.mathemagie import set_vector_length, add_point
from suturo_planning_search.map import Map

import utils
from utils import hex_to_color_msg
from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception import perception
from math import *
from suturo_msgs.msg import Task
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from suturo_interface_msgs.srv import SearchObjects, SearchObjectsResponse
import threading

class Interface(smach.State):

    search_object_server = None 

    #Gibt an, ob die Statemachine einmal durchlaufen wurde und die gefunden Objekte wiedergegeben werden sollen
    return_objects = False 

    def __init__(self):
        smach.State.__init__(self, outcomes=['cmdReceived','serviceBuild'],
                             input_keys=['yaml','enable_movement', 'objects_found','perceived_objects'],
                             output_keys=['yaml','enable_movement'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Interface')
        self.create_manipulation_object()
        self.create_node_service()

        #Ein Kommando wurde erhalten, verlasse den State und durchlaufe die Statemachine
        if self.search_object_server.cmd_received:
            self.search_object_server.cmd_received = False
            self.return_objects = True
            return 'cmdReceived'

        #Die Statemachine wurde durchlaufen. Es muessten nun Objekte gefunden worden sein, die der Service wiedergeben soll
        if self.return_objects:
            self.return_objects = False
            self.search_object_server.found_objects = userdata.objects_found
            self.search_object_server.event.set() #Wecke den SearchObjectServer auf, damit dieser die Objekte sendet

        #Kein start Kommando erhalten, bleibe im State
        rospy.sleep(2)
        return 'serviceBuild' 
        
    def create_node_service(self):
        """ Erstelle den benoetigten Service, falls diesr noch nicht vorhanden ist"""
        if self.search_object_server is None:
            self.search_object_server = SearchObjectServer()

    def create_manipulation_object(self):
        if utils.manipulation is None:
            #TODO throws exception! WHY !?!? 
            #utils.manipulation = Manipulation(userdata.yaml)
            rospy.sleep(2)


class SearchObjectServer(threading.Thread):
    """ Stellt einen Service zu Verfuegung, der ein Array von EurocObjects ausgibt. Implementierung als Thread, um den Service gezielt
    zu stoppen und nach Durchlauf der Statemachine zu starten, da Objekte nun gefunden wurden"""

    event = None #Das Event wird benoetigt um den Thread zu stoppten / starten
    cmd_received = False 
    found_objects = []

    def __init__(self):
        threading.Thread.__init__(self)
        #rospy.init_node('seach_objects_server') TODO throws exception
        service = rospy.Service('search_objects', SearchObjects, self.handle_search_objects) 
        self.event = threading.Event()
        
    def handle_search_objects(self, msg):
        self.cmd_received = True
        self.event.wait() #Lasse den Thread warten, bis die Statemachine durchlaufen ist
        return SearchObjectsResponse(result = self.found_objects)
    
    def run(self):
        pass

