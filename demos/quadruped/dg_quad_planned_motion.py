## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller, quad_com_control
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader

#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

#############################################################################
### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_com = Reader('ComReader')

filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_velocities_eff.dat"
filename_com = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_com.dat"
#filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"

# filename_pos = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/quadruped_positions_eff_rearing.dat"
# filename_vel = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/quadruped_velocities_eff_rearing.dat"

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_com)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_com.load(filename_com)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_com.selec.value = '1111'

des_pos = reader_pos.vector
des_vel = reader_vel.vector


################################################################################
des_fff = constVector([0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0],
                        "fff")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

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


p_gain_x = scale_values(slider_1, 300.0, "scale_kp_x")
p_gain_z = scale_values(slider_2, 300.0, "scale_kp_z")

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

###############################################################################

def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_com.rewind()
    reader_com.vector.recompute(0)

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp_split, des_pos, kd_split, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)


####################################### data logging ########################################

quad_com_ctrl = quad_com_control(robot)

quad_imp_ctrl.record_data()

robot.add_trace("PositionReader", "vector")
robot.add_ros_and_trace("PositionReader", "vector")

robot.add_trace("VelocityReader", "vector")
robot.add_ros_and_trace("VelocityReader", "vector")

robot.add_trace("ComReader", "vector")
robot.add_ros_and_trace("ComReader", "vector")

robot.add_trace("d_gain_split", "sout")
robot.add_ros_and_trace("d_gain_split", "sout")
