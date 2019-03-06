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

entityName = "hopper"
osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
osc_pos.setTimePeriod(0.001)
osc_pos.omega.value = 1.0*np.pi
osc_pos.magnitude.value = 0.06
osc_pos.phase.value = 0.0
osc_pos.bias.value = -0.22

osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
osc_vel.setTimePeriod(0.001)
osc_vel.omega.value = osc_pos.omega.value
osc_vel.magnitude.value = osc_pos.magnitude.value*osc_pos.omega.value
osc_vel.phase.value = osc_pos.phase.value + np.pi/2.0
osc_vel.bias.value = 0

unit_vector_pos = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_pos")
unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_vel")

des_pos = mul_double_vec_2(osc_pos.sout, unit_vector_pos, "des_position")
des_vel = mul_double_vec_2(osc_vel.sout, unit_vector_vel, "des_velocity")

########################################################################################################

add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 100
kp = add_kp.sout

add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.01
kd = add_kd.sout

# des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
# des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)
