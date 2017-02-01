#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import *

if __name__ == "__main__":
	rospy.wait_for_service('/Main/start')
	ts = rospy.ServiceProxy('/Main/start', Empty)
	print ts()
	