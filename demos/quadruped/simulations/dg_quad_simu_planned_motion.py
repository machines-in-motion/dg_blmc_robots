## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot
import time
import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

###### robot init #######################################################

# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.4
q[6] = 1.
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

###############################################################################




###############################################################################


pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
##For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout

control_torques = quad_impedance_controller(robot, pos_des, pos_des, pos_des, pos_des, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)



robot.run(10000, 1./60.)
