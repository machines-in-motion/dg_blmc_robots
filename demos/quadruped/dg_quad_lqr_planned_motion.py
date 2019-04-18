## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER
## and a center of mass controller

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader

#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

#############################################################################
### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_com = Reader('ComReader')
reader_lmom = Reader('Lmom')
reader_amom = Reader('Amom')
reader_forces = Reader('forces')


filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_velocities_eff.dat"
filename_com = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_com.dat"
filename_lmom = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_lmom.dat"
filename_amom = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_amom.dat"
filename_forces = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_forces.dat"

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_com)
file_exists(filename_lmom)
file_exists(filename_amom)
file_exists(filename_forces)


print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_com.load(filename_com)
reader_lmom.load(filename_lmom)
reader_amom.load(filename_amom)
reader_forces.load(filename_forces)


# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_com.selec.value = '1110'
reader_lmom.selec.value = '1110'
reader_amom.selec.value = '1110'
reader_forces.selec.value = '1111111111110'

des_pos = reader_pos.vector
des_vel = reader_vel.vector
des_com = reader_com.vector
des_lmom = reader_com.vector
des_amom = reader_amom.vector
des_forces = reader_forces.vector


##############################################################################
## Creating lqr gains through sliders for debugging
## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 400
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0/float(filter_size))
# we plug the centered sliders output to the input of the filter.
plug(robot.device.slider_positions, slider_filtered.sin)

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

com_kp = scale_values(slider_2, 60.0, "scale_com_kp")
com_kp_vec_x = mul_double_vec_2(com_kp, constVector([1.0, 0.0, 0.0], "unit_vec_com_kp_x"), "com_kp_x")
com_kp_vec_y = mul_double_vec_2(com_kp, constVector([0.0, 1.0, 0.0], "unit_vec_com_kp_y"), "com_kp_y")
com_kp_vec_z = mul_double_vec_2(com_kp, constVector([0.0, 0.0, 1.0], "unit_vec_com_kp_z"), "com_kp_z")

com_lmom_gain = scale_values(slider_3, 30.0, "scale_lmom_gain")
com_lmom_gain_vec_x = mul_double_vec_2(com_lmom_gain, constVector([1.0, 0.0, 0.0], "unit_vec_lmom_kp_x"), "com_lmom_kp_x")
com_lmom_gain_vec_y = mul_double_vec_2(com_lmom_gain, constVector([0.0, 1.0, 0.0], "unit_vec_lmom_kp_y"), "com_lmom_kp_y")
com_lmom_gain_vec_z = mul_double_vec_2(com_lmom_gain, constVector([0.0, 0.0, 1.0], "unit_vec_lmom_kp_z"), "com_lmom_kp_z")

com_amom_gain = scale_values(slider_4, 30.0, "scale_amom_gain")
com_amom_gain_vec_x = mul_double_vec_2(com_amom_gain, constVector([1.0, 0.0, 0.0], "unit_vec_amom_kp_x"), "com_amom_kp_x")
com_amom_gain_vec_y = mul_double_vec_2(com_amom_gain, constVector([0.0, 1.0, 0.0], "unit_vec_amom_kp_y"), "com_amom_kp_y")
com_amom_gain_vec_z = mul_double_vec_2(com_amom_gain, constVector([0.0, 0.0, 0.0], "unit_vec_amom_kp_z"), "com_amom_kp_z")

gains_leg_x_tmp = stack_two_vectors(com_kp_vec_x, com_lmom_gain_vec_x, 3, 3)
gains_leg_x = stack_two_vectors(gains_leg_x_tmp, com_amom_gain_vec_x, 6, 3)

gains_leg_y_tmp = stack_two_vectors(com_kp_vec_y, com_lmom_gain_vec_y, 3, 3)
gains_leg_y = stack_two_vectors(gains_leg_y_tmp, com_amom_gain_vec_y, 6, 3)

gains_leg_z_tmp = stack_two_vectors(com_kp_vec_z, com_lmom_gain_vec_z, 3, 3)
gains_leg_z = stack_two_vectors(gains_leg_z_tmp, com_amom_gain_vec_z, 6, 3)

## Converting into a 27d vector which when transformed to 3*9 vector represents
## feed forward forces for one leg
gains_lqr_tmp = stack_two_vectors(gains_leg_x, gains_leg_y, 9, 9)
gains_lqr_leg = stack_two_vectors(gains_lqr_tmp, gains_leg_z, 18, 9)
gains_lqr_leg = add_vec_vec(gains_lqr_leg, zero_vec(27, "tmp"), "gains_lqr_leg")

des_lqr_fl_fr = stack_two_vectors(gains_lqr_leg, gains_lqr_leg, 27, 27)
des_lqr_hl_hr = stack_two_vectors(gains_lqr_leg, gains_lqr_leg, 27, 27)

des_lqr = stack_two_vectors(des_lqr_fl_fr, des_lqr_hl_hr, 54, 54)


des_force = constVector([0.0, 0.0, (1.0)/4.0,
                       0.0, 0.0, (1.0)/4.0,
                       0.0, 0.0, (1.0)/4.0,
                       0.0, 0.0, (1.0)/4.0],
                        "fff")

###############################################################################

quad_com_ctrl = quad_com_control(robot)
f_lqr = quad_com_ctrl.return_lqr_tau(des_com, des_lmom, des_amom, des_forces, des_lqr)

# ################################################################################
des_fff = f_lqr

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

###############################################################################

slider_1_op = Component_of_vector("slider_1")
slider_1_op.setIndex(0)
plug(slider_filtered.sout, slider_1_op.sin)
slider_1 = slider_1_op.sout


p_gain_x = scale_values(slider_1, 300.0, "scale_kp_x")
p_gain_z = p_gain_x
unit_vector_x = constVector([1.0, 0.0, 0.0, 0.0, 0.0, 0.0], "unit_kp_x")
unit_vector_z = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_kp_z")

p_gain_x_6d = mul_double_vec_2(p_gain_x, unit_vector_x, "p_gain_x_to_6d")
p_gain_z_6d = mul_double_vec_2(p_gain_z, unit_vector_z, "p_gain_z_to_6d")

kp_split = add_vec_vec(p_gain_x_6d, p_gain_z_6d, "p_gain_split")

kd_split = constVector([0.8, 0.0, 0.8, 0.0, 0.0, 0.0], "kd_split")

# ###############################################################################
#
def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_com.rewind()
    reader_com.vector.recompute(0)
    reader_lmom.rewind()
    reader_lmom.vector.recompute(0)
    reader_amom.rewind()
    reader_amom.vector.recompute(0)
    # reader_forces.rewind()
    # reader_forces.vector.recompute(0)
    # reader_lqr1.rewind()
    # reader_lqr1.vector.recompute(0)
    # reader_lqr2.rewind()
    # reader_lqr2.vector.recompute(0)
    # reader_lqr2.rewind()
    # reader_lqr2.vector.recompute(0)

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp_split, des_pos, kd_split, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)


####################################### data logging ########################################
quad_imp_ctrl.record_data()

quad_com_ctrl.record_data()

robot.add_trace("PositionReader", "vector")
robot.add_ros_and_trace("PositionReader", "vector")

robot.add_trace("VelocityReader", "vector")
robot.add_ros_and_trace("VelocityReader", "vector")

robot.add_trace("ComReader", "vector")
robot.add_ros_and_trace("ComReader", "vector")

# robot.add_trace("d_gain_split", "sout")
# robot.add_ros_and_trace("d_gain_split", "sout")

robot.add_trace("gains_lqr_leg", "sout")
robot.add_ros_and_trace("gains_lqr_leg", "sout")
