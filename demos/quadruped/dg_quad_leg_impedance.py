## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller


#########################################################################################

# ## setting desired position
des_pos = constVector([0.0, 0.0, -0.25, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.25, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.25, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.25, 0.0, 0.0, 0.0],
                        "pos_des")

des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "vel_des")

##For making gain input dynamic through terminal
add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 150.0
kp = add_kp.sout

##For making gain input dynamic through terminal
add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.03
kd = add_kd.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

quad_imp_ctrl.record_data(record_vicon=True)

robot.add_trace("pos_des", "sout")
robot.add_ros_and_trace("pos_des", "sout")

robot.add_trace("vel_des", "sout")
robot.add_ros_and_trace("vel_des", "sout")
