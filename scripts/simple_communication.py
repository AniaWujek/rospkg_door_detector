#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def send_msg():
	pub = rospy.Publisher('subsystem_orders', String, queue_size=10)
	rospy.init_node('subsystem', anonymous=True)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		order = "time: %s" % rospy.get_time()
		#rospy.loginfo(order)
		pub.publish(order)
		rate.sleep()

if __name__ == '__main__':
	try:
		send_msg()
	except rospy.ROSInterruptException:
		pass
