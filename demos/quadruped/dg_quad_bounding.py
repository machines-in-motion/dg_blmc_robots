## This code for creating a stationary bound

## Author: Avadesh Meduri
## Date: 1/03/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller, quad_com_control
from leg_impedance_control.traj_generators import *




#########################################################################################

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

slider_4_op = Component_of_vector("slider_4")
slider_4_op.setIndex(3)
plug(slider_filtered.sout, slider_4_op.sin)
slider_4 = slider_4_op.sout


######################################################################################

##For making gain input dynamic through terminal
add_phase = Add_of_double('phase_op')
add_phase.sin1.value = 0.0
### Change this value for different gains
add_phase.sin2.value = 0.0
phase_zero = add_phase.sout

##For making gain input dynamic through terminal
add_phase_2 = Add_of_double('phase_op_2')
add_phase_2.sin1.value = 0.0
### Change this value for different gains
add_phase_2.sin2.value = np.pi
phase_pi = add_phase_2.sout


add_omega = Add_of_double('omega_op')
add_omega.sin1.value = 0.0
### Change this value for different gains
add_omega.sin2.value = 6.0*np.pi
omega = add_omega.sout


p_gain_x = scale_values(slider_1, 300.0, "scale_kp_x")
p_gain_z = scale_values(slider_2, 300.0, "scale_kp_z")
amplitude = scale_values(slider_3, 0.04, "amplitdue_scale")
stride = scale_values(slider_4, 0.1, "stride_value")


des_pos_fl, des_vel_fl = circular_trajectory_generator(stride, amplitude, omega , phase_zero, -0.22, "fl_des")
des_pos_fr, des_vel_fr = circular_trajectory_generator(stride, amplitude, omega, phase_zero, -0.22, "fr_des")
des_pos_hl, des_vel_hl = circular_trajectory_generator(stride, amplitude, omega , phase_pi, -0.22, "hl_des")
des_pos_hr, des_vel_hr = circular_trajectory_generator(stride, amplitude, omega, phase_pi, -0.22, "hr_des")

des_pos_fl_fr = stack_two_vectors(des_pos_fl, des_pos_fr, 6 , 6)
des_pos_hl_hr = stack_two_vectors(des_pos_hl, des_pos_hr, 6 , 6)

des_vel_fl_fr = stack_two_vectors(des_vel_fl, des_vel_fr, 6 , 6)
des_vel_hl_hr = stack_two_vectors(des_vel_hl, des_vel_hr, 6 , 6)

pos_des = stack_two_vectors(des_pos_fl_fr, des_pos_hl_hr, 12, 12)
pos_des = add_vec_vec(pos_des, zero_vec_24d, "pos_des")

vel_des = stack_two_vectors(des_vel_fl_fr, des_vel_hl_hr, 12, 12)
vel_des = add_vec_vec(vel_des, zero_vec_24d, "vel_des")

des_fff = constVector([0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0],
                        "fff")


#######################################################################################

# quad_com_ctrl = quad_com_control(robot)

## P_gain
unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_kp_x")
unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_kp_z")

p_gain_x_6d = mul_double_vec_2(p_gain_x, unit_vector_x, "p_gain_x_to_6d")
p_gain_z_6d = mul_double_vec_2(p_gain_z, unit_vector_z, "p_gain_z_to_6d")

p_gain_split = add_vec_vec(p_gain_x_6d, p_gain_z_6d, "p_gain_split")

kd_split = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split")

#######################################################################################

# kd = constVector([0.02, 0.0, 0.02, 0.0, 0.0, 0.0], "kd_split")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)

control_torques = quad_imp_ctrl.return_control_torques(p_gain_split, pos_des, kd_split, vel_des, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

quad_imp_ctrl.record_data()

quad_com_ctrl = quad_com_control(robot)

robot.add_trace("pos_des", "sout")
robot.add_ros_and_trace("pos_des", "sout")

robot.add_trace("vel_des", "sout")
robot.add_ros_and_trace("vel_des", "sout")

robot.add_trace("slider_fir_filter", "sout")
robot.add_ros_and_trace("slider_fir_filter", "sout")


###############################################################################
