#!/usr/bin/env python

import threading
from std_msgs.msg import *
import rospy
import numpy

door_homog_matrix_lock = threading.Lock()
door_homog_matrix = None

def array_elem(array, row, col):
	if array is not None:
		index = array.layout.dim[1].stride*row+col
		if index < len(array.data):
			return array.data[index]
	return None

def callback_door_homog(data):
	global door_homog_matrix
	door_homog_matrix_lock.acquire()
	door_homog_matrix = None
	door_homog_matrix = numpy.zeros(shape=(data.layout.dim[0].size,data.layout.dim[1].size))
	for row in range(0,data.layout.dim[0].size):
		for col in range(0,data.layout.dim[1].size):
			door_homog_matrix[row,col] = array_elem(data,row,col)
	print data
	print door_homog_matrix
	print "*********"
	door_homog_matrix_lock.release()

if __name__ == '__main__':
	rospy.init_node('receiver', anonymous=True)
	rospy.Subscriber("door_position", Float32MultiArray, callback_door_homog)
	while not rospy.is_shutdown():
		rospy.sleep(1.0)
		