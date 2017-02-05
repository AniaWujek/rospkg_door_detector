#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import *

if __name__ == "__main__":
	rospy.wait_for_service('/Door_localization/stop')
	rospy.ServiceProxy('/Door_localization/stop', Empty)
	rospy.wait_for_service('/Elements_localization/stop')
	rospy.ServiceProxy('/Elements_localization/stop', Empty)

	door_active = 0
	elements_active = 0

	print "SUBTASKS STOPPED"

	while(True):
		type = raw_input('D for door localization, E for elements localization, S for stop')
		if (type is 'D'):
			if (door_active==0):
				rospy.wait_for_service('/Elements_localization/stop')
				rospy.ServiceProxy('/Elements_localization/stop', Empty)
				rospy.wait_for_service('/Door_localization/start')
				rospy.ServiceProxy('/Door_localization/start', Empty)
				door_active = 1
				elements_active = 0
				print "DOOR LOCALIZATION ACTIVE"
		if (type='E'):
			if (elements_active==0):
				rospy.wait_for_service('/Door_localization/stop')
				rospy.ServiceProxy('/Door_localization/stop', Empty)
				rospy.wait_for_service('/Elements_localization/start')
				rospy.ServiceProxy('/Elements_localization/start', Empty)
				elements_active = 1
				door_active = 0
				print "ELEMENTS LOCALIZATION ACTIVE"
		if (type='S'):
			rospy.wait_for_service('/Door_localization/stop')
			rospy.ServiceProxy('/Door_localization/stop', Empty)
			rospy.wait_for_service('/Elements_localization/stop')
			rospy.ServiceProxy('/Elements_localization/stop', Empty)
			print "ALL SUBTASKS STOPPED"
	