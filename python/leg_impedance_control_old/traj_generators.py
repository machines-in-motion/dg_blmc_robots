## This code contains different trajectory generation functions

## Author: Avadesh Meduri
## Date: 6/02./2019

import numpy as np
from numpy import math
import dynamic_graph.sot.tools

from utils import *

#########################################################################


def mul_double_vec_2(doub, vec, entityName):
    ### need double as a signal
    mul = Multiply_double_vector(entityName)
    plug(doub, mul.signal('sin1'))
    plug(vec, mul.signal('sin2'))
    return mul.sout

###############################################################################

def linear_sine_generator(amplitude, omega, phase , bias ,entityName):
    ## generates a y = a*sin(W.t + phi)
    osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
    osc_pos.setTimePeriod(0.001)
    osc_pos.omega.value = omega*np.pi
    osc_pos.magnitude.value = amplitude
    osc_pos.phase.value = phase
    osc_pos.bias.value = bias

    osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
    osc_vel.setTimePeriod(0.001)
    osc_vel.omega.value = osc_pos.omega.value
    osc_vel.magnitude.value = osc_pos.magnitude.value*osc_pos.omega.value
    osc_vel.phase.value = osc_pos.phase.value + np.pi/2.0
    osc_vel.bias.value = 0

    unit_vector_pos = constVector([0.0, 0.0, 1.0], "unit_vector_pos")
    unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_vel")

    pos_traj = mul_double_vec_2(osc_pos.sout, unit_vector_pos, "des_position")
    vel_traj = mul_double_vec_2(osc_vel.sout, unit_vector_vel, "des_velocity")
    return pos_traj, vel_traj


def circular_trajectory_generator(radius, omega, phase, entityName):
    ## generates a circular circular_trajectory_generator

    osc_x = dynamic_graph.sot.tools.Oscillator(entityName + '_x')
    osc_x.setTimePeriod(0.001)
    osc_x.omega.value = omega*np.pi
    osc_x.magnitude.value = radius
    osc_x.bias.value = phase

    osc_y = dynamic_graph.sot.tools.Oscillator(entityName + '_y')
    osc_y.setTimePeriod(0.001)
    osc_y.omega.value = omega*np.pi
    osc_y.magnitude.value = radius
    osc_y.bias.value = phase + np.pi/2.0



#############################################################################
