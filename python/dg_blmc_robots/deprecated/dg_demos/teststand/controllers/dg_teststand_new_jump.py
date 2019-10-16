## simple jump on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values, sine_generator


#######################################################################################

## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 100
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

omega = mul_double_vec(5.0 , )
omega = scale_values(slider_1, 5.0, "omega_op")
amplitude = scale_values(slider_2, 0.15, "amplitdue_scale")

########################################################################################################
p_gain_z = Add_of_double('phase_op')
p_gain_z.sin1.value = 0.0
### Change this value for different gains
p_gain_z.sin2.value = 200.0
p_gain_z = p_gain_z.sout

##For making gain input dynamic through terminal
add_phase = Add_of_double('phase_op')
add_phase.sin1.value = 0.0
### Change this value for different gains
add_phase.sin2.value = 0.0
phase_zero = add_phase.sout
'''
add_omega = Add_of_double('omega_op')
add_omega.sin1.value = 0.0
### Change this value for different gains
add_omega.sin2.value = 3.6*np.pi
omega = add_omega.sout
'''

des_pos, des_vel = sine_generator(amplitude, omega, phase_zero, -0.14, "hopper")


########################################################################################################

unit_vec_kp = constVector([1.0, 0.0, 1.3, 0.0, 0.0, 0.0], "kp_unit_vec")

kp_split = mul_double_vec_2(p_gain_z, unit_vec_kp, "kp_6d")

kd_split = constVector([1.0, 0.0, 2.0, 0.0, 0.0, 0.0], "kd_6d")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0.0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

des_fff = constVector([0.0, 0.0, 0.8*9.81, 0.0, 0.0, 0.0], "des_fff")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)

des_new_pos = add_vec_vec(des_pos , constVector([-0.00, 0.0, 0.0, 0.0, 0.0, 0.0], "new_des_pos"))
control_torques = leg_imp_ctrl.return_control_torques(kp_split, des_new_pos,
                                                      kd_split, des_vel) #, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

###################### Record Data ##########################################################
'''
leg_imp_ctrl.record_data(robot)

robot.add_trace("hopper_des_position", "sout")
robot.add_ros_and_trace("hopper_des_position", "sout")

robot.add_trace("hopper_des_velocity", "sout")
robot.add_ros_and_trace("hopper_des_velocity", "sout")

robot.add_trace("slider_fir_filter", "sout")
robot.add_ros_and_trace("slider_fir_filter", "sout")
#slider 1:0.42089
#slider 2:0.57446
'''