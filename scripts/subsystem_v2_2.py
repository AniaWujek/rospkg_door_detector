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

	irpos = IRPOS("move", "Irp6ot", 7, "irp6ot_manager")

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
		

	actual_pose = irpos.get_cartesian_pose()

	# mnozenie macierzy:
	# p_base = T_base_d * p_d
	# T_base_d = T_base_tl6 * T_tl6_cam * T_cam_opt * T_opt_d

	a = raw_input("Kiedy odczytac macierz?")
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


	a = raw_input("kiedy odczytac punkty?")
	elements_points = elements_pos_points

	base_elements_points = []
	for i in range(0,elements_points.shape[0]):
		p = numpy.matrix([[elements_points[i,0]],[elements_points[i,1]-0.1],[0.6],[1.0]])
		base_elements_points.append(T_base_d * p)

	

	raw_input("kiedy rozpoczac ruch?")


	p0 = base_door_points[0]
	p1 = base_door_points[1]
	p2 = base_door_points[2]

	v0 = numpy.transpose(p0-p1)
	v1 = numpy.transpose(p2-p1)

	v0 = v0[0,0:3]
	v1 = v1[0,0:3]

	normal = numpy.cross(v1,v0)
	normal = normal/numpy.linalg.norm(normal)

	y = numpy.cross(normal,[0,1,0])
	x = numpy.cross(y,normal)
	z = normal

	rot = numpy.concatenate((x,y,z),axis=0)
	rot = numpy.transpose(rot)
	

	b = numpy.matrix([0,0,0])
	rot = numpy.concatenate((rot,b),axis=0)
	b = numpy.matrix([[0],[0],[0],[1]])
	rot = numpy.concatenate((rot,b),axis=1)

	print rot

	q = quaternion_from_matrix(rot)

	element = 3

	j_position = irpos.get_joint_position()
	track_position = j_position[0]
	dist = track_position - base_elements_points[element][1]
	print dist
	new_track = 1.06 + float(base_elements_points[element][1])
	if new_track<0.0:
		new_track = 0.0
	new_j_position = ((new_track,)+ j_position[1:7])
	print new_j_position
	irpos.move_to_joint_position(new_j_position, 10)
	point = Point(base_elements_points[element][0],base_elements_points[element][1],base_elements_points[element][2])

	new_pose = Pose()
	new_pose.position = point
	new_pose.orientation.x = q[0]
	new_pose.orientation.y = q[1]
	new_pose.orientation.z = q[2]
	new_pose.orientation.w = q[3]

	print new_pose

	irpos.move_to_cartesian_pose(10.0, new_pose)
		

