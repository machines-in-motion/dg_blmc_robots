## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2

#########################################################################################

## setting desired position
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

des_fff = constVector([0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0],
                        "fff")

#######################################################################################

kp = constVector([50.0, 0.0, 50.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

# quad_imp_ctrl.record_data(record_vicon=False)
#
# robot.add_trace("pos_des", "sout")
# robot.add_ros_and_trace("pos_des", "sout")
#
# robot.add_trace("vel_des", "sout")
# robot.add_ros_and_trace("vel_des", "sout")
