# Impedance controller for quadruped
# Author : Avadesh Meduri
# Date : 23 Jan 2019


import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

from dynamic_graph import plug
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


from dynamic_graph.sot.core.control_pd import ControlPD

# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

q = zero(robot.pin_robot.nq)
print("value", q)


robot.run(5000, 1./60.)
