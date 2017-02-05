#!/usr/bin/env python

import threading
from std_msgs.msg import *
import rospy
import numpy
import sys
from std_srvs.srv import *


# mutexy dla zmiennych, do ktorych zapisywane sa dane z wirtualnego receptora
door_homog_matrix_lock = threading.Lock()
door_pos_points_lock = threading.Lock()
elements_pos_points_lock = threading.Lock()
element_homog_matrix_lock = threading.Lock()


# zmienne, do ktorych zapisywane sa dane z wirtualnego receptora
door_homog_matrix = None
door_pos_points = None
elements_pos_points = None
element_homog_matrix = None


# flagi okreslajace, czy wartosci w zmiennych sa swieze
door_homog_matrix_fresh = False
door_pos_points_fresh = False
elements_pos_points_fresh = False
element_homog_matrix_fresh = False


# odczytanie elementu z Float32MultiArray
def array_elem(array, row, col):
	if array is not None:
		index = array.layout.dim[1].stride*row+col
		if index < len(array.data):
			return array.data[index]
	return None

# wpisanie Float32MultiArray do macierzy
def read_data(data):
	m = numpy.zeros(shape=(data.layout.dim[0].size,data.layout.dim[1].size))
	for row in range(0,data.layout.dim[0].size):
		for col in range(0,data.layout.dim[1].size):
			m[row,col] = array_elem(data,row,col)
	return m


# funkcje odbierajace dane, gdy na topicu pojawia sie nowe dane
def callback_door_pos(data):
	print "callback 1"
	global door_pos_points
	global door_pos_points_fresh
	door_pos_points_lock.acquire()
	door_pos_points = read_data(data)
	door_pos_points_fresh = True
	door_pos_points_lock.release()

def callback_door_homog(data):
	print "callback 2"
	global door_homog_matrix
	global door_homog_matrix_fresh
	door_homog_matrix_lock.acquire()
	door_homog_matrix = read_data(data)
	door_homog_matrix_fresh = True
	door_homog_matrix_lock.release()

def callback_elements_pos(data):
	print "callback 3"
	global elements_pos_points
	global elements_pos_points_fresh
	elements_pos_points_lock.acquire()
	elements_pos_points = read_data(data)
	elements_pos_points_fresh = True
	elements_pos_points_lock.release()

def callback_element_homog(data):
	print "callback 4"
	global element_homog_matrix
	global element_homog_matrix_fresh
	element_homog_matrix_lock.acquire()
	element_homog_matrix = read_data(data)
	element_homog_matrix_fresh = True
	element_homog_matrix_lock.release()


if __name__ == '__main__':

	rospy.init_node('receiver', anonymous=True)

	# utworzenie subscriberow
	rospy.Subscriber("door_homog_matrix", Float32MultiArray, callback_door_homog)
	rospy.Subscriber("door_position", Float32MultiArray, callback_door_pos)
	rospy.Subscriber("door_elements_position", Float32MultiArray, callback_elements_pos)
	rospy.Subscriber("element_homog_matrix", Float32MultiArray, callback_element_homog)

	# wylaczenie taskow na wszelki wypadek
	rospy.wait_for_service('/Door/stop')
	ts = rospy.ServiceProxy('/Door/stop', Empty)
	ts()
	rospy.wait_for_service('/Elements/stop')
	ts = rospy.ServiceProxy('/Elements/stop', Empty)
	ts()

	# flagi ktory task jest aktywny
	door_active = 0
	elements_active = 0

	print "SUBTASKS STOPPED"

	while(True):
		type = raw_input('D for door localization, E for elements localization, S for stop: ')
		if (type is 'D'):
			if (door_active is 0):
				rospy.wait_for_service('/Elements/stop')
				ts = rospy.ServiceProxy('/Elements/stop', Empty)
				ts()
				rospy.wait_for_service('/Door/start')
				ts = rospy.ServiceProxy('/Door/start', Empty)
				ts()
				door_active = 1
				elements_active = 0
				print "DOOR LOCALIZATION ACTIVE"
		if (type is 'E'):
			if (elements_active is 0):
				rospy.wait_for_service('/Door/stop')
				ts = rospy.ServiceProxy('/Door/stop', Empty)
				ts()
				rospy.wait_for_service('/Elements/start')
				ts = rospy.ServiceProxy('/Elements/start', Empty)
				ts()
				elements_active = 1
				door_active = 0
				print "ELEMENTS LOCALIZATION ACTIVE"
		if (type is 'S'):
			rospy.wait_for_service('/Door/stop')
			ts = rospy.ServiceProxy('/Door/stop', Empty)
			ts()
			rospy.wait_for_service('/Elements/stop')
			ts = rospy.ServiceProxy('/Elements/stop', Empty)
			ts()
			print "ALL SUBTASKS STOPPED"

		print "************** data **************"
		if door_homog_matrix_fresh:
			door_homog_matrix_fresh = False
			print door_homog_matrix

		if door_pos_points_fresh:
			door_pos_points_fresh = False
			print door_pos_points

		if elements_pos_points_fresh:
			elements_pos_points_fresh = False
			print elements_pos_points

		if element_homog_matrix_fresh:
			element_homog_matrix_fresh = False
			print element_homog_matrix	
		
		print "**************"
		