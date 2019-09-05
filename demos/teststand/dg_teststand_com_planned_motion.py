## This is code to track desired trajectories from the planner
## The gains have to be tuned with the sliders
## and leg impedance

##Author : Elham


from leg_impedance_control.utils import *
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

from os.path import join

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

kp = scale_values(slider_1, 100.0, "scale_kp")#######
kd = scale_values(slider_2, 25.0, "scale_kd")########


unit_vec_101 = constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vec_101")


kp = mul_double_vec_2(kp, unit_vec_101, "kp")
kd = mul_double_vec_2(kd, unit_vec_101, "kd")


################################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

from dynamic_graph.sot.core.reader import Reader

reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_fff_com = Reader('FeedForwardForceComReader')

filename_pos = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_positions_eff.dat")
filename_vel = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_velocities_eff.dat")
filename_fff_com = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_centroidal_forces.dat")

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_fff_com)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_fff_com.load(filename_fff_com)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '000000000000000000111111'
reader_vel.selec.value = '000000000000000000111111'
reader_fff_com.selec.value = '1110'

des_pos = reader_pos.vector
des_vel = reader_vel.vector
#des_fff = reader_fff_com.vector

###############################################################################

def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_fff_com.rewind()
    reader_fff_com.vector.recompute(0)

###############################################################################

add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

leg_imp_ctrl = leg_impedance_controller("hopper")
plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)

control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, None)

plug(control_torques, robot.device.ctrl_joint_torques)

