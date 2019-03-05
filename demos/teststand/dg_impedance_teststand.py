## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

#######################################################################################

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig

#######################################################################################

add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 100
kp = add_kp.sout

add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.01
kd = add_kd.sout

des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)
