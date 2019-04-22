#impedance controller implementation for COM (used for quadruped)
#Author : Avadesh Meduri
#Date : 25/03/19


from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import *

################################################################################

kp_com = constVector([50.0, 0.0, 50.0], "kp_com")
kd_com = constVector([0.5, 0.0, 0.5], "kd_com")
kp_ang_com = constVector([1.0, 1.0, 1.0], "kp_ang_com")
des_pos_com = constVector([0.0, 0.0, 0.0], "des_pos_com")
des_vel_com = constVector([0.0, 0.0, 0.0], "des_vel_com")
des_fff_com = constVector([0.0, 0.0, 2.2*9.81], "des_fff_com")
des_omega = constVector([0.0, 0.0, 0.0], "des_com_omega")
des_fft_com = constVector([0.0, 0.0, 0.0], 'des_fft_com')

pos_des = constVector([0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0],
                        "des_pos")

vel_des = zero_vec(24, "des_vel")

###############################################################################

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


p_gain_x = scale_values(slider_1, 250.0, "scale_kp_x")
p_gain_z = scale_values(slider_2, 250.0, "scale_kp_z")

d_gain_x = scale_values(slider_3, 0.8, "scale_kd_x")
d_gain_z = scale_values(slider_4, 0.8, "scale_kd_z")


unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_kp_x")
unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_kp_z")

p_gain_x_6d = mul_double_vec_2(p_gain_x, unit_vector_x, "p_gain_x_to_6d")
p_gain_z_6d = mul_double_vec_2(p_gain_z, unit_vector_z, "p_gain_z_to_6d")

kp_split = add_vec_vec(p_gain_x_6d, p_gain_z_6d, "p_gain_split")

unit_vector_xd = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_kd_x")
unit_vector_zd = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_kd_z")

d_gain_x_6d = mul_double_vec_2(d_gain_x, unit_vector_xd, "d_gain_x_to_6d")
d_gain_z_6d = mul_double_vec_2(d_gain_z, unit_vector_zd, "d_gain_z_to_6d")

kd_split = add_vec_vec(d_gain_x_6d, d_gain_z_6d, "d_gain_split")

#################################################################################

quad_com_ctrl = quad_com_control(robot)
com_tau = quad_com_ctrl.compute_torques(kp_com, des_pos_com, kd_com, des_vel_com,
                                                                    des_fff_com)

tau_per_leg = quad_com_ctrl.return_com_torques(com_tau)

ang_tau = quad_com_ctrl.compute_ang_control_torques(kp_ang_com, des_omega_com, des_fft_com)

#################################################################################
##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp_split, pos_des,
                                                kd_split, vel_des, kf, tau_per_leg)
# plug(control_torques, robot.device.ctrl_joint_torques)



################################################################################
quad_com_ctrl.record_data()
