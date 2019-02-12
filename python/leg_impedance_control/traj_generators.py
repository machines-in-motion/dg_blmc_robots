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
    osc = dynamic_graph.sot.tools.Oscillator(entityName)
    osc.setTimePeriod(0.001)
    osc.omega.value = omega *np.pi
    osc.magnitude.value = amplitude
    osc.phase.value = phase
    osc.bias.value = bias

    return osc.sout


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
