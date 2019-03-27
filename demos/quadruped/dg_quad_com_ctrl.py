#impedance controller implementation for COM (used for quadruped)
#Author : Avadesh Meduri
#Date : 25/03/19


from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control



################################################################################

kp = constVector([1.0, 1.0, 1.0], "kp")
kd = constVector([1.0, 1.0, 1.0], "kd")
des_pos = constVector([0.0, 0.0, 0.0], "des_pos")
des_vel = constVector([0.0, 0.0, 0.0], "des_vel")
des_fff = constVector([0.0, 0.0, 0.0], "des_fff")
###############################################################################

quad_com_imp_ctrl = quad_com_control(robot)
# #

quad_com_imp_ctrl.set_bias()

tau = quad_com_imp_ctrl.compute_torques(kp, des_pos, kd, des_vel, des_fff)

###############################################################################

robot.add_trace("quad_com_ctrl", "tau")
robot.add_ros_and_trace("quad_com_ctrl", "tau")
