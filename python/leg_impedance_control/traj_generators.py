## This code contains different trajectory generation functions

## Author: Avadesh Meduri
## Date: 6/02./2019

import numpy as np
from numpy import math
import dynamic_graph.sot.tools

from leg_impedance_control.utils import *

#########################################################################


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

###############################################################################

def sine_generator(amplitude, omega, phase , bias ,entityName):
    ## generates a y = a*sin(W.t + phi)
    osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
    osc_pos.setTimePeriod(0.001)
    plug(omega, osc_pos.omega)
    plug(amplitude, osc_pos.magnitude)
    plug(phase, osc_pos.phase)
    osc_pos.bias.value = bias

    osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
    osc_vel.setTimePeriod(0.001)
    plug(osc_pos.omega, osc_vel.omega)
    plug(osc_pos.magnitude, osc_vel.magnitude)
    plug(add_doub_doub(osc_pos.phase.value, np.pi/2.0, "phase_add").sout, osc_vel.phase )
    osc_vel.bias.value = 0

    unit_vector_pos = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_pos")
    unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_vel")

    pos_traj = mul_double_vec_2(osc_pos.sout, unit_vector_pos, "des_position")
    vel_traj = mul_double_vec_2(osc_vel.sout, unit_vector_vel, "des_velocity")
    return pos_traj, vel_traj


def circular_trajectory_generator(radius_x, radius_z, omega, phase, entityName):
    ## generates a circular circular_trajectory_generator

    ## Position################################################################

    osc_x = dynamic_graph.sot.tools.Oscillator(entityName + '_x')
    osc_x.setTimePeriod(0.001)
    osc_x.omega.value = omega*np.pi
    osc_x.magnitude.value = radius_x
    osc_x.bias.value = 0.0
    osc_x.phase.value = phase

    osc_z = dynamic_graph.sot.tools.Oscillator(entityName + '_y')
    osc_z.setTimePeriod(0.001)
    osc_z.omega.value = osc_x.omega.value
    osc_z.magnitude.value = radius_z
    osc_z.bias.value = -.23
    osc_z.phase.value = osc_x.phase.value + np.pi/2.0

    unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_vector_x")
    unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_z")

    pos_des_x = mul_double_vec_2(osc_x.sout, unit_vector_x, entityName + "sine_des_position_x")
    pos_des_z = mul_double_vec_2(osc_z.sout, unit_vector_z, entityName + "sine_des_position_z")

    pos_des = add_vec_vec(pos_des_x, pos_des_z, entityName + "_des_pos")

    ## Velocity#################################################################

    osc_xd = dynamic_graph.sot.tools.Oscillator(entityName + '_xd')
    osc_xd.setTimePeriod(0.001)
    osc_xd.omega.value = osc_x.omega.value
    osc_xd.magnitude.value = osc_x.magnitude.value * osc_xd.omega.value
    osc_xd.bias.value = 0.0
    osc_xd.phase.value = phase + np.pi/2.0

    osc_zd = dynamic_graph.sot.tools.Oscillator(entityName + '_xd')
    osc_zd.setTimePeriod(0.001)
    osc_zd.omega.value = osc_z.omega.value
    osc_zd.magnitude.value = osc_z.magnitude.value * osc_zd.omega.value
    osc_zd.bias.value = 0.0
    osc_zd.phase.value = osc_zd.phase.value + np.pi/2.0

    unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_vector_x1")
    unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_z1")

    vel_des_x = mul_double_vec_2(osc_xd.sout, unit_vector_x, entityName + "sine_des_velocity_x")
    vel_des_z = mul_double_vec_2(osc_zd.sout, unit_vector_z, entityName + "sine_des_velocity_z")

    vel_des = add_vec_vec(vel_des_x, vel_des_z, entityName + "_des_vel")

    return pos_des, vel_des


#############################################################################
