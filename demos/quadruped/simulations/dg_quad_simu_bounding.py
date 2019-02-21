## quadruped bounding

## Author: Avadesh Meduri
## Date: 6/02/2019

from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

###### robot init #######################################################

# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.4
q[6] = 1.
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)


########################################################################


osc_x = dynamic_graph.sot.tools.Oscillator("circular_fl_fr" + '_x')
osc_x.setTimePeriod(0.001)
osc_x.omega.value = 2.5*np.pi
osc_x.magnitude.value = 0.1
osc_x.bias.value = -0.03
osc_x.phase.value = 0.0

osc_z = dynamic_graph.sot.tools.Oscillator("circular_fl_fr" + '_y')
osc_z.setTimePeriod(0.001)
osc_z.omega.value = osc_x.omega.value
osc_z.magnitude.value = 0.08
osc_z.bias.value = -.2
osc_z.phase.value = 0.0 + np.pi/2.0

osc_x_h = dynamic_graph.sot.tools.Oscillator("circular_hl_hr" + '_x')
osc_x_h.setTimePeriod(0.001)
osc_x_h.omega.value = osc_x.omega.value
osc_x_h.magnitude.value = osc_x.magnitude.value
osc_x_h.bias.value = osc_x.bias.value
osc_x_h.phase.value = osc_x.phase.value + np.pi

osc_z_h = dynamic_graph.sot.tools.Oscillator("circular_hl_hr" + '_y')
osc_z_h.setTimePeriod(0.001)
osc_z_h.omega.value = osc_x.omega.value
osc_z_h.magnitude.value = osc_z.magnitude.value
osc_z_h.bias.value = osc_z.bias.value
osc_z_h.phase.value = osc_z.phase.value + np.pi

###########################################################################

unit_vector_x = constVector([1.0, 0.0, 0.0], "unit_vector_x")
unit_vector_z = constVector([0.0, 0.0, 1.0], "unit_vector_z")

pos_des_fl_fr_x = mul_double_vec_2(osc_x.sout, unit_vector_x, "sine_des_position_front_x")
pos_des_fl_fr_z = mul_double_vec_2(osc_z.sout, unit_vector_z, "sine_des_position_front_z")

pos_des_hl_hr_x = mul_double_vec_2(osc_x.sout, unit_vector_x, "sine_des_position_hind_x")
pos_des_hl_hr_z = mul_double_vec_2(osc_z.sout, unit_vector_z, "sine_des_position_hind_z")

pos_des_fl_fr = add_vec_vec(pos_des_fl_fr_x, pos_des_fl_fr_z, "ellipse_traj_fl_fr")
pos_des_hl_hr = add_vec_vec(pos_des_hl_hr_x, pos_des_hl_hr_z, "ellipse_traj_hl_hr")


# For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout

control_torques = quad_impedance_controller(robot, pos_des_fl_fr, pos_des_fl_fr, pos_des_hl_hr, pos_des_hl_hr, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)



robot.run(10000, 1./60.)