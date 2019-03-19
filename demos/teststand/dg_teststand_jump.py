## simple jump on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import *


#######################################################################################

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

# p_gain_z = scale_values(slider_1, 200.0, "scale_kp_z")
amplitude = scale_values(slider_1, 0.07, "amplitdue_scale")

########################################################################################################

##For making gain input dynamic through terminal
add_phase = Add_of_double('phase_op')
add_phase.sin1.value = 0.0
### Change this value for different gains
add_phase.sin2.value = 0.0
phase_zero = add_phase.sout

add_omega = Add_of_double('omega_op')
add_omega.sin1.value = 0.0
### Change this value for different gains
add_omega.sin2.value = 2.0*np.pi
omega = add_omega.sout


des_pos, des_vel = sine_generator(amplitude, omega, phase_zero, -0.22, "hopper")


########################################################################################################

kp_split = constVector([150.0, 0.0, 250.0, 0.0, 0.0, 0.0], "kp_split")

kd_split = constVector([0.4, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split")

# des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
# des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp_split, des_pos, kd_split, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

###################### Record Data ##########################################################

leg_imp_ctrl.record_data(robot)

robot.add_trace("hopper_des_position", "sout")
robot.add_ros_and_trace("hopper_des_position", "sout")

robot.add_trace("hopper_des_velocity", "sout")
robot.add_ros_and_trace("hopper_des_velocity", "sout")

robot.add_trace("slider_fir_filter", "sout")
robot.add_ros_and_trace("slider_fir_filter", "sout")
