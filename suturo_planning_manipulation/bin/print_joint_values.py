#!/usr/bin/env python
import rospy

from suturo_planning_manipulation.manipulation import Manipulation
from suturo_planning_perception.perception import *


if __name__ == '__main__':
    # print (lambda x: a(2, x))(3)
    rospy.init_node('head_mover', anonymous=True)
    #
    mani = Manipulation()

    print mani.get_arm_base_move_group().get_current_joint_values()