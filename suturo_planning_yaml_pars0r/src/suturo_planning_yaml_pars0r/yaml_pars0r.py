#!/usr/bin/env python
import yaml, rospy, threading
from suturo_msgs.msg import task


class Yaml_pars0r:

    def __init__(self):
        self._yaml = None
        self._lock = threading.Lock()
        self.pub = rospy.Publisher('yaml_pars0r', task, queue_size=10)
        rospy.init_node('yaml_pars0r_node', anonymous=True)
        r = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                message = self.parse_yaml()
                rospy.loginfo(message)
                self.pub.publish(message)
                r.sleep()
        except rospy.ROSInterruptException:
            pass

    def parse_yaml(self):
        msg = task(description='ololo')
        return msg

    @property
    def yaml(self):
        self._lock.acquire()
        tmp = self._yaml
        self._lock.release()
        return tmp


    @yaml.setter
    def yaml(self, value):
        self._lock.acquire()
        self._yaml = value
        self._lock.release()