#!/usr/bin/env python

import threading
from std_msgs.msg import *
import rospy
import numpy

door_homog_matrix_lock = threading.Lock()
door_pos_points_lock = threading.Lock()
elements_pos_points_lock = threading.Lock()

door_homog_matrix = None
door_pos_points = None
elements_pos_points = None

def array_elem(array, row, col):
	if array is not None:
		index = array.layout.dim[1].stride*row+col
		if index < len(array.data):
			return array.data[index]
	return None

def read_data(data):
	m = numpy.zeros(shape=(data.layout.dim[0].size,data.layout.dim[1].size))
	for row in range(0,data.layout.dim[0].size):
		for col in range(0,data.layout.dim[1].size):
			m[row,col] = array_elem(data,row,col)
	return m

def callback_door_pos(data):
	global door_pos_points
	door_pos_points_lock.acquire()
	door_pos_points = read_data(data)
	door_pos_points_lock.release()

def callback_door_homog(data):
	global door_homog_matrix
	door_homog_matrix_lock.acquire()
	door_homog_matrix = read_data(data)
	door_homog_matrix_lock.release()

def callback_elements_pos(data):
	global elements_pos_points
	elements_pos_points_lock.acquire()
	elements_pos_points = read_data(data)
	elements_pos_points_lock.release()

if __name__ == '__main__':
	rospy.init_node('receiver', anonymous=True)
	rospy.Subscriber("door_homog_matrix", Float32MultiArray, callback_door_homog)
	rospy.Subscriber("door_position", Float32MultiArray, callback_door_pos)
	rospy.Subscriber("door_elements_position", Float32MultiArray, callback_elements_pos)
	while not rospy.is_shutdown():
		rospy.sleep(1.0)
		print door_homog_matrix
		print door_pos_points
		print elements_pos_points
		print "**************"
		