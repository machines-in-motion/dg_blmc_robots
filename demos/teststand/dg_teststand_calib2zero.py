## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

import numpy as np
import pinocchio as se3
from pinocchio.utils import zero
from dynamic_graph import plug

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig

from dynamic_graph_manager.dg_tools import Calibrator
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph_manager.dg_tools import ControlPD
from dynamic_graph.sot.core.operator import Stack_of_vector
from leg_impedance_control.utils import *

################################################################################

#### Simulation Setup
# Get the robot corresponding to the quadruped.
robot = get_teststand_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)
robot.set_gravity((0.0,0.0,0.0))
#### Calibrator setup

joint_calibrator = Calibrator('joint_calibrator')

## plug inputs in
plug(robot.device.joint_positions, joint_calibrator.rawPosition)
plug(robot.device.joint_velocities, joint_calibrator.velocity)
joint_calibrator.desiredVelocity.value = [0.3, 0] # 2*[0.0,]
joint_calibrator.kp.value = 2*[8.0,]
joint_calibrator.hardstop2zero.value = 2*[0.0,]

#### create a switch for the output torque

control_switch = SwitchVector("control_switch")
control_switch.setSignalNumber(2) # we want to switch between 2 signals
plug(joint_calibrator.control, control_switch.sin0)
# plug(pd.control, control_switch.sin1)
control_switch.sin1.value = [0.0,0.0]
control_switch.selection.value = 0 # pick and switch manually for now

plug(control_switch.sout, robot.device.ctrl_joint_torques)

#################################################################################
