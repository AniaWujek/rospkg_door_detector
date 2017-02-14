#!/usr/bin/env python

from irpos import *
import sys

if __name__ == '__main__':
    irpos = IRPOS("observe_door", "Irp6ot", 7, "irp6ot_manager")

    if len(sys.argv)>2:
    	time = float(sys.argv[2])
    	if time>4.0:
	    	if sys.argv[1] == 'synchro':
	    		irpos.move_to_synchro_position(time)
	    	if sys.argv[1] == 'observe':
	    		irpos.move_to_joint_position([ 0.45, -0.5 * math.pi, -0.55 * math.pi, -0.02*math.pi, -0.16 * math.pi, 1.5 * math.pi, -0.5 * math.pi], time)
	    	if sys.argv[1] == 'calib':
	    		irpos.move_to_joint_position([ 0, 0, -0.5 * math.pi, -0.07*math.pi, -0.1 * math.pi, 1.8 * math.pi, -0.8* math.pi], time)
	    	if sys.argv[1] == 'test':
	    		irpos.move_to_cartesian_pose(time, Pose(Point(0.12,-1.6,0.87), irpos.get_cartesian_pose().orientation))
	    	if sys.argv[1] == 'pose':
	    		print irpos.get_cartesian_pose()