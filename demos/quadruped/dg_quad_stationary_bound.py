## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller, quad_com_control
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values, sine_generator


quad_imp_ctrl = quad_leg_impedance_controller(robot)

#########################################################################################

# ## setting desired position
pos_bias = constVector([0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
                        "pos_bias")

unit_vector_pos_12d = constVector([0.0, 0.0, .16, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.16, 0.0, 0.0, 0.0,],
                       "unit_vector_pos_12d")

zero_vec_24d = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "zero_vec")

## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 400
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0/float(filter_size))
# we plug the centered sliders output to the input of the filter.
plug(robot.device.slider_positions, slider_filtered.sin)

slider_1_op = Component_of_vector("slider_1")
slider_1_op.setIndex(0)
plug(slider_filtered.sout, slider_1_op.sin)
slider_1 = slider_1_op.sout

slider_2_op = Component_of_vector("slider_2")
slider_2_op.setIndex(1)
plug(slider_filtered.sout, slider_2_op.sin)
slider_2 = slider_2_op.sout

slider_3_op = Component_of_vector("slider_3")
slider_3_op.setIndex(2)
plug(slider_filtered.sout, slider_3_op.sin)
slider_3 = slider_3_op.sout

######################################################################################

##For making gain input dynamic through terminal
add_phase = Add_of_double('phase_op')
add_phase.sin1.value = 0.0
### Change this value for different gains
add_phase.sin2.value = 0.0
phase_zero = add_phase.sout

add_omega = Add_of_double('omega_op')
add_omega.sin1.value = 0.0
### Change this value for different gains
add_omega.sin2.value = 2.5*np.pi
omega = add_omega.sout


#omega = scale_values(slider_1, 1.5, "omega_scale")
amplitude = scale_values(slider_2, 0.07, "amplitdue_scale")
phase = scale_values(slider_3, np.pi, "phase_scale")

des_pos_fl, des_vel_fl = sine_generator(amplitude, omega , phase_zero, -0.21, "fl_des")
des_pos_fr, des_vel_fr = sine_generator(amplitude, omega, phase_zero, -0.21, "fr_des")
des_pos_hl, des_vel_hl = sine_generator(amplitude, omega , phase, -0.21, "hl_des")
des_pos_hr, des_vel_hr = sine_generator(amplitude, omega, phase, -0.21, "hr_des")

des_pos_fl_fr = stack_two_vectors(des_pos_fl, des_pos_fr, 6 , 6)
des_pos_hl_hr = stack_two_vectors(des_pos_hl, des_pos_hr, 6 , 6)

des_vel_fl_fr = stack_two_vectors(des_vel_fl, des_vel_fr, 6 , 6)
des_vel_hl_hr = stack_two_vectors(des_vel_hl, des_vel_hr, 6 , 6)

pos_des = stack_two_vectors(des_pos_fl_fr, des_pos_hl_hr, 12, 12)
pos_des = add_vec_vec(pos_des, zero_vec_24d, "pos_des")

vel_des = stack_two_vectors(des_vel_fl_fr, des_vel_hl_hr, 12, 12)
vel_des = add_vec_vec(vel_des, zero_vec_24d, "vel_des")


#######################################################################################

# ##For making gain input dynamic through terminal
# add_kp = Add_of_double('kp')
# add_kp.sin1.value = 0
# ### Change this value for different gains
# add_kp.sin2.value = 50.0
# kp = add_kp.sout
#
#
# com_des_pos = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
#                        "com_des_pos")
#
#
# quad_com_ctrl = quad_com_control(robot)
# quad_com_ctrl.return_com_control_torques(kp, com_des_pos)
# quad_com_ctrl.record_data()

#######################################################################################

p_gain = scale_values(slider_1, 160.0, "scale_kp")



##For making gain input dynamic through terminal
add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.02
kd = add_kd.sout

control_torques = quad_imp_ctrl.return_control_torques(p_gain, pos_des, kd, vel_des)

#plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

quad_imp_ctrl.record_data(record_vicon=False)

robot.add_trace("pos_des", "sout")
robot.add_ros_and_trace("pos_des", "sout")

robot.add_trace("vel_des", "sout")
robot.add_ros_and_trace("vel_des", "sout")

robot.add_trace("slider_fir_filter", "sout")
robot.add_ros_and_trace("slider_fir_filter", "sout")


###############################################################################
