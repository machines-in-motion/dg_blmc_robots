# Impedance controller for quadruped taking independent jacobians for each leg for standing
# Author : Avadesh Meduri
# Date : 29 Jan 2019


from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
##########################################################################################





# ## setting desired position
pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
##For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout

control_torques = quad_impedance_controller(robot, pos_des, pos_des, pos_des, pos_des, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)





############################################################
