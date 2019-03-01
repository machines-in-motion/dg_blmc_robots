## This code is a simulation for a sinemotion using the impedance controller

## Author: Avadesh Meduri
## Date: 12/02./2019


import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot
from leg_impedance_control.traj_generators import *

import pinocchio as se3
from pinocchio.utils import zero

from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
##########################################################################################
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

#########################################################################################

des_pos, des_vel = linear_sine_generator(0.08, 3.5, 0.0 , -.2, "hopper")

pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
# For making gain input dynamic through terminal
kp = Add_of_double('Kd')
kp.sin1.value = 0
### Change this value for different gains
kp.sin2.value = 100.0
Kp = kp.sout

# For making gain input dynamic through terminal
kd = Add_of_double('Kd')
kd.sin1.value = 0
### Change this value for different gains
kd.sin2.value = 10.0
Kd = kp.sout



control_torques = quad_impedance_controller(robot, des_pos, des_pos, des_pos, des_pos, Kp)

plug(control_torques, robot.device.ctrl_joint_torques)











#########################################################################################

robot.run(10000, 1./60.)
