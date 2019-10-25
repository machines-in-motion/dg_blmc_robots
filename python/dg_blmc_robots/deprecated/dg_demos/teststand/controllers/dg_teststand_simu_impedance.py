## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

#######################################################################################

from dg_blmc_robots.teststand import get_teststand_robot
from dg_blmc_robots.teststand import TeststandConfig

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

#######################################################################################

kp_split = constVector([100.0, 0.0, 150.0, 0.0, 0.0, 0.0], "kp_split")


kd_split = constVector([2.0, 0.0, 2.0, 0.0, 0.0, 0.0], "kd_split")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

des_fff = constVector([0.0, 0.0, 0.8*9.81, 0.0, 0.0, 0.0], "des_fff")

des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp_split, des_pos,
                                                    kd_split, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)
#########################################################################

robot.run(10000, 1./60.)
