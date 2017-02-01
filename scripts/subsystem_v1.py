#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from door_detector.msg import Order_simple

def send_msg():
	pub = rospy.Publisher('subsystem_orders', Order_simple, queue_size=10)
	rospy.init_node('subsystem', anonymous=True)
	rate = rospy.Rate(1)
	order = Order_simple()
	order.behaviour = "door_localization"
	order.img_path = ""
	order.models_path = ""
	while not rospy.is_shutdown():
		#rospy.loginfo(order)
		pub.publish(order)
		rate.sleep()

if __name__ == '__main__':

	


	try:
		send_msg()
	except rospy.ROSInterruptException:
		pass
