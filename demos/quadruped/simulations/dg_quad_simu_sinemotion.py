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


pos_des_fl_fr_z = linear_sine_generator(0.06, 1.0, 0.0 , -0.2, "fl")
pos_des_hl_hr_z = linear_sine_generator(0.06, 1.0, 0.0 , -0.2, "hl")

###########################################################################


unit_vector = constVector([0.0, 0.0, 1.0], "unit_vector")

pos_des_fl_fr = mul_double_vec_2(pos_des_fl_fr_z, unit_vector, "sine_des_position_front")
pos_des_hl_hr = mul_double_vec_2(pos_des_hl_hr_z, unit_vector, "sine_des_position_hind")

pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
# For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout

control_torques = quad_impedance_controller(robot, pos_des_fl_fr, pos_des_fl_fr, pos_des_hl_hr, pos_des_hl_hr, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)











#########################################################################################

robot.run(10000, 1./60.)
