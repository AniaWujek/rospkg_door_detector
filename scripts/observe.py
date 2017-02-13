#!/usr/bin/env python

from irpos import *

if __name__ == '__main__':
    irpos = IRPOS("observe_door", "Irp6ot", 7, "irp6ot_manager")
    
    irpos.move_to_joint_position([ 0, -0.5 * math.pi, -0.5 * math.pi, -0.05*math.pi, -0.1 * math.pi, 1.5 * math.pi, -0.5 * math.pi], 10.0)
    #irpos.move_to_synchro_position(10.0)
