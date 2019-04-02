## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller, quad_com_control
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.switch import SwitchVector


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


#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

def read_planned_traj(path, EntityName):
    reader_pos = Reader('PositionReader_' + EntityName )
    reader_vel = Reader('VelocityReader_' + EntityName)
    reader_com = Reader('ComReader_' + EntityName)

    filename_pos = path + "quadruped_positions_eff.dat"
    filename_vel = path + "quadruped_velocities_eff.dat"
    filename_com = path + "quadruped_com.dat"

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

    return reader_pos, reader_vel, reader_com

#############################################################################

fpath_jump = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/SAB/jump/"
fpath_trot = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/SAB/trot/"
fpath_fjump = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/SAB/forward_jump/"


rpos_jump, rvel_jump, rcom_jump = read_planned_traj(fpath_jump , "jump")
rpos_trot, rvel_trot, rcom_trot = read_planned_traj(fpath_trot, "trot")
rpos_fjump, rvel_fjump, rcom_fjump = read_planned_traj(fpath_fjump, "fjump")


control_switch_pos = SwitchVector("control_switch_pos")
control_switch_pos.setSignalNumber(3) # we want to switch between 2 signals
plug(rpos_jump.vector, control_switch_pos.sin0)
plug(rpos_trot.vector, control_switch_pos.sin1)
plug(rpos_fjump.vector, control_switch_pos.sin2)
control_switch_pos.selection.value = 0 # pick and switch manually

control_switch_vel = SwitchVector("control_switch_vel")
control_switch_vel.setSignalNumber(3) # we want to switch between 2 signals
plug(rvel_jump.vector, control_switch_vel.sin0)
plug(rvel_trot.vector, control_switch_vel.sin1)
plug(rvel_fjump.vector, control_switch_vel.sin2)

control_switch_vel.selection.value = 0 # pick and switch manually

#############################################################################

def start_jump():
    rpos_jump.rewind()
    rpos_jump.vector.recompute(0)
    rvel_jump.rewind()
    rvel_jump.vector.recompute(0)
    rcom_jump.rewind()
    rcom_jump.vector.recompute(0)

    control_switch_pos.selection.value = 0
    control_switch_vel.selection.value = 0

def start_trot():
    rpos_trot.rewind()
    rpos_trot.vector.recompute(0)
    rvel_trot.rewind()
    rvel_trot.vector.recompute(0)
    rcom_trot.rewind()
    rcom_trot.vector.recompute(0)

    control_switch_pos.selection.value = 1
    control_switch_vel.selection.value = 1

def start_fjump():
    rpos_fjump.rewind()
    rpos_fjump.vector.recompute(0)
    rvel_fjump.rewind()
    rvel_fjump.vector.recompute(0)
    rcom_fjump.rewind()
    rcom_fjump.vector.recompute(0)

    control_switch_pos.selection.value = 2
    control_switch_vel.selection.value = 2




###############################################################################

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp_split, control_switch_pos.sout,
                                            kd_split, control_switch_vel.sout, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)


####################################### data logging ########################################
#
# # quad_com_ctrl = quad_com_control(robot)
#
# quad_imp_ctrl.record_data()
#
# robot.add_trace("PositionReader", "vector")
# robot.add_ros_and_trace("PositionReader", "vector")
#
# robot.add_trace("VelocityReader", "vector")
# robot.add_ros_and_trace("VelocityReader", "vector")
#
# robot.add_trace("ComReader", "vector")
# robot.add_ros_and_trace("ComReader", "vector")
#
# robot.add_trace("d_gain_split", "sout")
# robot.add_ros_and_trace("d_gain_split", "sout")
