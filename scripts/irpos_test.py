#!/usr/bin/env python

from irpos import *

if __name__ == '__main__':
    irpos = IRPOS("testowy", "Irp6ot", 7, "irp6ot_manager")    
    print('[Joint position]')
    print str(irpos.get_joint_position())
    print('[Motor position]')
    print str(irpos.get_motor_position())
    print('[Cartesian pose]')
    print str(irpos.get_cartesian_pose())
    print('[Wrench]')
    print str(irpos.get_force_readings())
    print "Irp6ot get_status test completed"
