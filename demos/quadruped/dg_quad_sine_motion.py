## Make the legs follow a sine motion(quadruped)

## Author: Avadesh Meduri
## Date: 6/02/2019

from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

########################################################################

## generates a y = a*sin(W.t + phi)
osc = dynamic_graph.sot.tools.Oscillator('sine')
osc.setTimePeriod(0.001)
osc.omega.value = 1.0 *np.pi
osc.magnitude.value = 0.06
osc.phase.value = 0.0
osc.bias.value = -0.2


# pos_des_fl_fr_z = linear_sine_generator(0.06, des_vel, 0.0 , -0.2, "fl")
# pos_des_hl_hr_z = linear_sine_generator(0.06, des_vel, 0.0 , -0.2, "hl")


###########################################################################


unit_vector = constVector([0.0, 0.0, 1.0], "unit_vector")

# pos_des_fl_fr = mul_double_vec_2(pos_des_fl_fr_z, unit_vector, "sine_des_position_front")
# pos_des_hl_hr = mul_double_vec_2(pos_des_hl_hr_z, unit_vector, "sine_des_position_hind")

pos_des_fl_fr = mul_double_vec_2(osc.sout, unit_vector, "sine_des_position_front")
pos_des_hl_hr = mul_double_vec_2(osc.sout, unit_vector, "sine_des_position_hind")


pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
# For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 50.0
gain_value = add.sout

control_torques = quad_impedance_controller(robot, pos_des_fl_fr, pos_des_fl_fr, pos_des_hl_hr, pos_des_hl_hr, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)

##################################
