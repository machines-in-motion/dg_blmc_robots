## This code contains different trajectory generation functions

## Author: Avadesh Meduri
## Date: 6/02./2019

import numpy as np
from numpy import math
import dynamic_graph.sot.tools

from leg_impedance_control.utils import *

#########################################################################

def add_doub_doub_2(db1, db2, entityName):
    add = Add_of_double(entityName)
    plug(db1, add.signal('sin1'))
    plug(db2, add.signal('sin2'))
    return add.sout


def mul_double_vec_2(doub, vec, entityName):
    ### need double as a signal
    mul = Multiply_double_vector(entityName)
    plug(doub, mul.signal('sin1'))
    plug(vec, mul.signal('sin2'))
    return mul.sout

def scale_values(double, scale, entityName):
    mul = Multiply_of_double(entityName)
    mul.sin0.value = scale
    plug(double, mul.sin1)
    return mul.sout

def mul_doub_doub(db1, db2, entityName):
    dif1 = Multiply_of_double(entityName)
    plug(db1, dif1.sin0)
    plug(db2, dif1.sin1)
    return dif1.sout

##########################################################

##For making gain input dynamic through terminal
add_pi = Add_of_double('pi')
add_pi.sin1.value = 0
### Change this value for different gains
add_pi.sin2.value = np.pi/2.0
pi = add_pi.sout

###############################################################################

def sine_generator(amplitude, omega, phase , bias ,entityName):
    ## generates a y = a*sin(W.t + phi)
    osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
    osc_pos.setTimePeriod(0.001)
    plug(omega, osc_pos.omega)
    plug(amplitude, osc_pos.magnitude)
    plug(phase, osc_pos.phase)
    # osc_pos.phase.value = phase
    osc_pos.bias.value = bias

    osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
    osc_vel.setTimePeriod(0.001)
    plug(osc_pos.omega, osc_vel.omega)
    plug(osc_pos.magnitude, osc_vel.magnitude)
    plug(add_doub_doub_2(osc_pos.phase, pi, entityName + "phase_add"), osc_vel.phase )
    osc_vel.bias.value = 0

    unit_vector_pos = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "_unit_vector_pos")
    unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "_unit_vector_vel")

    pos_traj = mul_double_vec_2(osc_pos.sout, unit_vector_pos, entityName + "_des_position")
    vel_traj = mul_double_vec_2(osc_vel.sout, unit_vector_vel, entityName + "_des_velocity")

    return pos_traj, vel_traj

def circular_trajectory_generator(radius_x, radius_z, omega, phase, bias, entityName):
    ## generates a circular circular_trajectory_generator

    ## Position################################################################

    osc_x = dynamic_graph.sot.tools.Oscillator(entityName + "_posx")
    osc_x.setTimePeriod(0.001)
    plug(omega, osc_x.omega)
    plug(radius_x, osc_x.magnitude)
    plug(phase, osc_x.phase)
    osc_x.bias.value = 0.0

    osc_xd = dynamic_graph.sot.tools.Oscillator(entityName + '_velx')
    osc_xd.setTimePeriod(0.001)
    plug(osc_x.omega, osc_xd.omega)
    plug(mul_doub_doub(osc_x.omega, osc_x.magnitude, "dx"), osc_xd.magnitude)
    #plug(osc_x.magnitude, osc_xd.magnitude)
    plug(add_doub_doub_2(osc_x.phase, pi, entityName + "phase_addx"), osc_xd.phase )
    osc_xd.bias.value = 0

    osc_z = dynamic_graph.sot.tools.Oscillator(entityName + "_posz")
    osc_z.setTimePeriod(0.001)
    plug(omega, osc_z.omega)
    plug(radius_z, osc_z.magnitude)
    plug(add_doub_doub_2(osc_x.phase, pi, entityName + "phase_addx"), osc_z.phase)
    osc_z.bias.value = bias

    osc_zd = dynamic_graph.sot.tools.Oscillator(entityName + '_velz')
    osc_zd.setTimePeriod(0.001)
    plug(osc_z.omega, osc_zd.omega)
    plug(mul_doub_doub(osc_z.omega, osc_z.magnitude, "dz"), osc_zd.magnitude)
    # plug(osc_z.magnitude, osc_zd.magnitude)
    plug(add_doub_doub_2(osc_z.phase, pi, entityName + "phase_addz"), osc_zd.phase )
    osc_zd.bias.value = 0


    unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], entityName + "unit_vector_x")
    unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "unit_vector_z")

    pos_des_x = mul_double_vec_2(osc_x.sout, unit_vector_x, entityName + "sine_des_position_x")
    pos_des_z = mul_double_vec_2(osc_z.sout, unit_vector_z, entityName + "sine_des_position_z")

    pos_des = add_vec_vec(pos_des_x, pos_des_z, entityName + "_des_pos")

    unit_vector_xd = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], entityName + "unit_vector_xd")
    unit_vector_zd = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], entityName + "unit_vector_zd")

    vel_des_x = mul_double_vec_2(osc_xd.sout, unit_vector_xd, entityName + "sine_des_velocity_x")
    vel_des_z = mul_double_vec_2(osc_zd.sout, unit_vector_zd, entityName + "sine_des_velocity_z")

    vel_des = add_vec_vec(vel_des_x, vel_des_z, entityName + "_des_vel")

    return pos_des, vel_des


#############################################################################
