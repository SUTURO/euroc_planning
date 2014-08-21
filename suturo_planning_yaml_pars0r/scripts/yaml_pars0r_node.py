#!/usr/bin/env python
from suturo_planning_yaml_pars0r.yaml_pars0r import YamlPars0r
import rospy
import signal
import time


def main():
    rospy.init_node('yaml_pars0r_node', anonymous=True, log_level=rospy.INFO)
    y = YamlPars0r()
    signal.signal(signal.SIGINT, lambda sig, frame: y.kill(sig, frame))
    while True:
        time.sleep(0.05)

if __name__ == "__main__":
    main()