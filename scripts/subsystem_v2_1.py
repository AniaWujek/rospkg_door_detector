#!/usr/bin/env python

import threading
from std_msgs.msg import *
import rospy
import numpy
import sys
from std_srvs.srv import *
from irpos import *
import math
from transformations import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


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

door_homog_counter = 0


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
	global door_pos_points
	global door_pos_points_fresh
	door_pos_points_lock.acquire()
	door_pos_points = read_data(data)
	door_pos_points_fresh = True
	door_pos_points_lock.release()

def callback_door_homog(data):
	global door_homog_matrix
	global door_homog_matrix_fresh
	global door_homog_counter
	door_homog_matrix_lock.acquire()
	door_homog_matrix = read_data(data)
	door_homog_matrix_fresh = True
	if door_homog_counter<100:
		door_homog_counter = door_homog_counter+1
	door_homog_matrix_lock.release()

def callback_elements_pos(data):
	global elements_pos_points
	global elements_pos_points_fresh
	elements_pos_points_lock.acquire()
	elements_pos_points = read_data(data)
	elements_pos_points_fresh = True
	elements_pos_points_lock.release()

def callback_element_homog(data):
	global element_homog_matrix
	global element_homog_matrix_fresh
	element_homog_matrix_lock.acquire()
	element_homog_matrix = read_data(data)
	element_homog_matrix_fresh = True
	element_homog_matrix_lock.release()


if __name__ == '__main__':

	#rospy.init_node('receiver', anonymous=True)

	

	# utworzenie subscriberow
	rospy.Subscriber("door_homog_matrix", Float32MultiArray, callback_door_homog)
	rospy.Subscriber("door_position", Float32MultiArray, callback_door_pos)
	rospy.Subscriber("door_elements_position", Float32MultiArray, callback_elements_pos)
	rospy.Subscriber("element_homog_matrix", Float32MultiArray, callback_element_homog)

	publisher = rospy.Publisher('door_marker_array', Marker)
	publisher2 = rospy.Publisher('elements_marker_array', Marker)

	irpos = IRPOS("door", "Irp6ot", 7, "irp6ot_manager")

	# # wylaczenie taskow na wszelki wypadek
	# rospy.wait_for_service('/Door/stop')
	# ts = rospy.ServiceProxy('/Door/stop', Empty)
	# ts()
	# rospy.wait_for_service('/Elements/stop')
	# ts = rospy.ServiceProxy('/Elements/stop', Empty)
	# ts()

	# # flagi ktory task jest aktywny
	# door_active = 0
	# elements_active = 0

	# print "SUBTASKS STOPPED"
	# rospy.wait_for_service('/Elements/stop')
	# ts = rospy.ServiceProxy('/Elements/stop', Empty)
	# ts()
	# rospy.wait_for_service('/Door/start')
	# ts = rospy.ServiceProxy('/Door/start', Empty)
	# ts()

	print "DOOR LOCALIZATION ACTIVE"

	while not rospy.is_shutdown():
		

		actual_pose = irpos.get_cartesian_pose()

		# mnozenie macierzy:
		# p_base = T_base_d * p_d
		# T_base_d = T_base_tl6 * T_tl6_cam * T_cam_opt * T_opt_d

		door_homog_matrix_lock.acquire()
		T_opt_d = door_homog_matrix
		door_homog_matrix_lock.release()

		T_cam_opt = numpy.matrix([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])
		T_tl6_cam = numpy.matrix([[0,-1,0,-0.0551],[1,0,0,0],[0,0,1,0.13],[0,0,0,1]])
		pose = irpos.get_cartesian_pose()
		qx = pose.orientation.x
		qy = pose.orientation.y
		qz = pose.orientation.z
		qw = pose.orientation.w
		px = pose.position.x
		py = pose.position.y
		pz = pose.position.z
		quaternion = [qx, qy, qz, qw]

		T_base_tl6 = quaternion_matrix(quaternion) + numpy.matrix([[0,0,0,px],[0,0,0,py],[0,0,0,pz],[0,0,0,0]])

		T_base_d = T_base_tl6 * T_tl6_cam * T_cam_opt * T_opt_d

		door_points = door_pos_points

		base_door_points = []
		for i in range(0,door_points.shape[0]):
			p = numpy.matrix([[door_points[i,0]],[door_points[i,1]],[0.0],[1.0]])
			base_door_points.append(T_base_d * p)

		marker = Marker()
		marker.header.frame_id = "/tl_base"
		marker.type = marker.TRIANGLE_LIST
		marker.action = marker.ADD
		marker.scale.x = 1.0
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.color.a = 1.0
		marker.color.g = 1.0
		marker.pose.orientation.w = 1.0
		p = Point()
		p.x = base_door_points[0][0]
		p.y = base_door_points[0][1]
		p.z = base_door_points[0][2]		
		
		p1 = Point()
		p1.x = base_door_points[1][0]
		p1.y = base_door_points[1][1]
		p1.z = base_door_points[1][2]
		
		p2 = Point()
		p2.x = base_door_points[2][0]
		p2.y = base_door_points[2][1]
		p2.z = base_door_points[2][2]
	
		p3 = Point()
		p3.x = base_door_points[3][0]
		p3.y = base_door_points[3][1]
		p3.z = base_door_points[3][2]
	
		marker.points.append(p)
		marker.points.append(p1)
		marker.points.append(p2)
		marker.points.append(p3)
		marker.points.append(p2)
		marker.points.append(p)

		publisher.publish(marker)






		elements_points = elements_pos_points

		base_elements_points = []
		for i in range(0,elements_points.shape[0]):
			p = numpy.matrix([[elements_points[i,0]],[elements_points[i,1]],[0.05],[1.0]])
			base_elements_points.append(T_base_d * p)

		marker2 = Marker()
		marker2.header.frame_id = "/tl_base"
		marker2.type = marker.SPHERE_LIST
		marker2.action = marker.ADD
		marker2.scale.x = 0.1
		marker2.scale.y = 0.1
		marker2.scale.z = 0.1
		marker2.color.a = 1.0
		marker2.color.r = 1.0
		marker2.pose.orientation.w = 1.0

		for i in range(0,len(base_elements_points)):
			p =Point()
			p.x = base_elements_points[i][0]
			p.y = base_elements_points[i][1]
			p.z = base_elements_points[i][2]

			marker2.points.append(p)

		publisher.publish(marker)
		publisher2.publish(marker2)


		
		
		
		rospy.sleep(1.0)
