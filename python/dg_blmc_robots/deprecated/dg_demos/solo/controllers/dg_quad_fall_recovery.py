## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values, sine_generator


quad_imp_ctrl = quad_leg_impedance_controller(robot)

#########################################################################################

# ## setting desired position
pos_bias = constVector([0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.3, 0.0, 0.0, 0.0],
                        "pos_bias")

pos_bias_x = constVector([-0., 0.0, -0.0, 0.0, 0.0, 0.0,
                       -0., 0.0, -0.0, 0.0, 0.0, 0.0,
                       -0., 0.0, -0.0, 0.0, 0.0, 0.0,
                       -0., 0.0, -0.0, 0.0, 0.0, 0.0],
                        "pos_bias_x")

unit_vector_pos_12d = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0, 0.0, 0.0,],
                       "unit_vector_pos_12d")

unit_vector_pos_12dx = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       1.0, 0.0, 0.0, 0.0, 0.0, 0.0,],
                       "unit_vector_pos_12d_x")

zero_vec_24d = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "zero_vec")

## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 200
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


slider_1 = scale_values(slider_1, 0.6, "des_pos_fl_fr")
slider_2 = scale_values(slider_2, 0.6, "des_pos_hl_hr")
slider_3 = scale_values(slider_3, 0.6, "des_pos_fl_fr_x")
slider_4 = scale_values(slider_4, 0.6, "des_pos_hl_hr_x")
######################################################################################

des_pos_fl_fr = mul_double_vec_2(slider_1, unit_vector_pos_12d, "l1")
des_pos_hl_hr = mul_double_vec_2(slider_1, unit_vector_pos_12d, "l2")

des_pos_fl_fr_x = mul_double_vec_2(slider_3, unit_vector_pos_12dx, "l1x")
des_pos_hl_hr_x = mul_double_vec_2(slider_4, unit_vector_pos_12dx, "l2x")


pos_des = stack_two_vectors(des_pos_fl_fr, des_pos_hl_hr, 12, 12)
pos_des_x = stack_two_vectors(des_pos_fl_fr_x, des_pos_hl_hr_x, 12, 12)
pos_des_x = add_vec_vec(pos_des_x, pos_bias_x, "pos_des_x")


pos_des = add_vec_vec(pos_des, pos_bias, "pos_des_z")
pos_des = add_vec_vec(pos_des_x, pos_des, "pos_des")

vel_des = zero_vec_24d

#######################################################################################
add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 0.0
kp = add_kp.sout

add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.5
kd = add_kd.sout

kp = mul_double_vec_2(kp,  constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit"), "kp_split")
kd = mul_double_vec_2(kd,  constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit"), "kd_split")

control_torques = quad_imp_ctrl.return_control_torques(kp, pos_des, kd, zero_vec_24d)

plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

# quad_imp_ctrl.record_data(record_vicon=False)

robot.add_trace("pos_des", "sout")
robot.add_ros_and_trace("pos_des", "sout")
