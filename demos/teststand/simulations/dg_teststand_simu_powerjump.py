## Simulation for power jump with teststand
## Author : Avadesh Meduri
## Date : 14/04/2019


from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

from dynamic_graph_manager.dg_tools import power_jump_control

#######################################################################################

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero


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

###############################################################################

des_weight_fff = constVector([0.0, 0.0, 0.8*9.81, 0.0, 0.0, 0.0], "des_weight_fff")
des_fff = constVector([0.0, 0.0, 2.0*9.81, 0.0, 0.0, 0.0], "des_fff")
des_pos_trigger = constVector([0.0, 0.0, -0.1, 0.0, 0.0, 0.0],"pos_des_trigger")
des_pos_air = constVector([0.0, 0.0, -0.25, 0.0, 0.0, 0.0],"pos_des_air")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

des_kp_ground = constVector([0.5, 0.0, 100.0, 0.0, 0.0, 0.0],"des_kp_ground")
des_kp_air = constVector([0.5, 0.0, 100.0, 0.0, 0.0, 0.0],"des_kp_air")
des_kd = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0],"des_kd_air")


##############################################################################

power_jump_ctrl = power_jump_control("teststand")

###############################################################################

robot.run(10000, 1./60.)
