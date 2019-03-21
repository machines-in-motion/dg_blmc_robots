## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

#######################################################################################


#######################################################################################

kp_split = constVector([250.0, 0.0, 250.0, 0.0, 0.0, 0.0], "kp_split")

kd_split = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split")

des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp_split, des_pos, kd_split, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

###################### Record Data ##########################################################
leg_imp_ctrl.record_data(robot)
