# Code to carry out balancing task on a planck by solo
#Author : Avadesh Meduri
#Date : 25/04/19


from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values
from dynamic_graph_manager.vicon_sdk import ViconClientEntity


#############################################################################

# ## filter slider_value
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


################################################################################

# # ### calculating leg length of FL and HL
# joint_positions = robot.device.signal("joint_positions")
# joint_velocities = robot.device.signal("joint_velocities")
#
# fl = leg_impedance_controller("balance_fl")
#
# jp_fl = selec_vector(joint_positions, 0, 2, 'position_slicer_fl_tmp')
# jv_fl = selec_vector(joint_velocities, 0, 2, 'velocity_slicer_fl_tmp')
#
# plug(stack_zero(jp_fl,"add_base_fl_pos"), fl.robot_dg.position)
# plug(stack_zero(jv_fl, "add_base_fl_vel"), fl.robot_dg.velocity )
#
# fl_ll = fl.return_leg_length()
#
# hl = leg_impedance_controller("balance_hl")
#
# jp_hl = selec_vector(joint_positions, 4, 6, 'position_slicer_hl_tmp')
# jv_hl = selec_vector(joint_velocities, 4, 6, 'velocity_slicer_hl_tmp')
#
# plug(stack_zero(jp_hl,"add_base_hl_pos"), hl.robot_dg.position)
# plug(stack_zero(jv_hl, "add_base_hl_vel"), hl.robot_dg.velocity )
#
#
# hl_ll = hl.return_leg_length()
#
#
# #
# ###############################################################################
kp_com = constVector([0.0, 0.0, 150.0], "kp_com")
kd_com = constVector([25.0, 0.0, 20.0], "kd_com")
kp_ang_com = constVector([200.0, 200.0, 0.0], "kp_ang_com")
kd_ang_com = constVector([25.0, 25.0, 0.0], "kd_ang_com")
des_pos_com = constVector([0.0, 0.0, 0.26], "des_pos_com")
des_fff_com = constVector([0.0, 0.0, 2.17784*9.81], "des_fff_com")
des_ori_com = constVector([0.0, 0.0, 0.0, 1.0], "des_com_ori")
des_amom_com = constVector([0.0, 0.0, 0.0], "des_com_amom")
des_fft_com = constVector([0.0, 0.0, 0.0], 'des_fft_com')

vel_des = zero_vec(24, "des_vel")


###############################################################################



unit_vec_100 = constVector([2.17784, 0.0, 0.0], "unit_vec_100")

add_vel = Add_of_double('vel')
add_vel.sin1.value = 0.0
### Change this value for different gains
add_vel.sin2.value = 0.0
des_vel = add_vel.sout


des_lmom_com = mul_double_vec_2(des_vel, unit_vec_100)


###############################################################################

def gen_r_matrix(rx, ry, rz):
    R = np.matrix([[0, -rz, ry],
                   [rz, 0, -rx],
                   [-ry, rx, 0]])
    return R


###############################################################################

I = np.matrix([[1.0 , 0.0 , 0.0],
               [0.0 , 1.0 , 0.0],
               [0.0 , 0.0 , 1.0]])

r_fl = gen_r_matrix(0.2, 0.15, 0.0)
r_fr = gen_r_matrix(0.2, -0.15, 0.0)
r_hl = gen_r_matrix(-0.2, 0.15, 0.0)
r_hr = gen_r_matrix(-0.2, -0.15, 0.0)

zero_matrix = np.zeros((3,3))

py_ce = np.block([[I, I, I, I, -I, zero_matrix],
                  [r_fl, r_fr, r_hl, r_hr, zero_matrix, -I]])

py_ce = np.asarray(py_ce)

w1 = 1.0
w2 = 1.0
py_hess = np.zeros((18,18))
np.fill_diagonal(py_hess, w1)

py_reg = np.zeros((18,18))
np.fill_diagonal(py_reg, 0.0001)

hess = constMatrix(py_hess, "hess")
reg = constMatrix(py_reg, "regularizer")
g0 = zero_vec(18, "g0")
ce = constMatrix(py_ce, "ce")
ce0 = zero_vec(6, "ce0")
ci = constMatrix(np.zeros((6,18)), "ci")
ci0 = zero_vec(6, "ci0")
###############################################################################
quad_com_ctrl = quad_com_control(robot, ViconClientEntity, "solo")

# plug(fl_ll, quad_com_ctrl.com_imp_ctrl.leg_length_fl)
# plug(hl_ll, quad_com_ctrl.com_imp_ctrl.leg_length_hl)
# des_pos_com = quad_com_ctrl.com_imp_ctrl.compute_des_com_pos

lctrl = quad_com_ctrl.compute_torques(kp_com, des_pos_com, kd_com, des_lmom_com,
                                                                    des_fff_com)
actrl = quad_com_ctrl.compute_ang_control_torques(kp_ang_com, des_ori_com, kd_ang_com, des_amom_com, des_fft_com)

com_torques = quad_com_ctrl.return_com_torques(lctrl, actrl, hess, g0, ce, ci, ci0, reg)

############################################################################


###############################################################################
add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 100.0
kp = add_kp.sout


kp = mul_double_vec_2(kp,  constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit"), "kp_split")
kd = constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "kd_split")

## setting desired position
des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.2, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.2, 0.0, 0.0, 0.0],
                        "pos_des")

des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        "vel_des")

des_fff = com_torques

add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

###############################################################################
quad_com_ctrl.record_data()
