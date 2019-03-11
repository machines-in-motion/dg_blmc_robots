## simple jump on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import *
#######################################################################################

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig

#######################################################################################

# entityName = "hopper"
# osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
# osc_pos.setTimePeriod(0.001)
# osc_pos.omega.value = 1.0*np.pi
# osc_pos.magnitude.value = 0.06
# osc_pos.phase.value = 0.0
# osc_pos.bias.value = -0.22
#
# osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
# osc_vel.setTimePeriod(0.001)
# osc_vel.omega.value = osc_pos.omega.value
# osc_vel.magnitude.value = osc_pos.magnitude.value*osc_pos.omega.value
# osc_vel.phase.value = osc_pos.phase.value + np.pi/2.0
# osc_vel.bias.value = 0
#
# unit_vector_pos = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_pos")
# unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_vel")
#
# des_pos = mul_double_vec_2(osc_pos.sout, unit_vector_pos, "des_position")
# des_vel = mul_double_vec_2(osc_vel.sout, unit_vector_vel, "des_velocity")


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

amp_op = multiply_of_double("amp")
amp_op.sin1.value = 0.6
plug(slider_1, amp_op.sin2)
amp = amp_op.sout.value

omega_op = multiply_of_double("omega")
omega_op.sin1.value = 0.6
plug(slider_1, omega_op.sin2)
omage = omega_op.sout.value

des_pos, des_vel = linear_sine_generator(amp, omega, 0.0, -0.22, "hopper")

########################################################################################################

add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 0
kp = add_kp.sout

add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.0
kd = add_kd.sout

# des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
# des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

#plug(control_torques, robot.device.ctrl_joint_torques)

################### tracking ################################################

robot.add_trace("pos_des", "sout")
robot.add_ros_and_trace("pos_des", "sout")

# robot.add_trace("vel_des", "sout")
# robot.add_ros_and_trace("vel_des", "sout")

robot.add_trace("slider_fir_filter", "sout")
robot.add_ros_and_trace("slider_fir_filter", "sout")
