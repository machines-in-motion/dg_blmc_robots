## quadruped bounding

## Author: Avadesh Meduri
## Date: 6/02/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller
from leg_impedance_control.traj_generators import circular_trajectory_generator

from pinocchio.utils import zero

import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_quadruped_robot



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
# (radius, omega, phase, entityName)
pos_des_fl, vel_des_fl = circular_trajectory_generator(0.037, 0.06, 3.75, 0.0, "traj_fl" )
pos_des_fr, vel_des_fr = circular_trajectory_generator(0.037, 0.06, 3.75, 0.0, "traj_fr" )
pos_des_hl, vel_des_hl = circular_trajectory_generator(0.037, 0.06, 3.75, np.pi + np.pi/12, "traj_hl" )
pos_des_hr, vel_des_hr = circular_trajectory_generator(0.037, 0.06, 3.75, np.pi + np.pi/12, "traj_hr" )

pos_des_1 = stack_two_vectors(pos_des_fl, pos_des_fr, 6, 6)
pos_des_2 = stack_two_vectors(pos_des_hl, pos_des_hr, 6, 6)
des_pos = stack_two_vectors(pos_des_1, pos_des_2, 12, 12)

vel_des_1 = stack_two_vectors(vel_des_fl, vel_des_fr, 6, 6)
vel_des_2 = stack_two_vectors(vel_des_hl, vel_des_hr, 6, 6)
des_vel = stack_two_vectors(vel_des_1, vel_des_2, 12, 12)

##################################################################################
##For maki0ng gain input dynamic through terminal
add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 200.0
kp = add_kp.sout

##For making gain input dynamic through terminal
add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.02
kd = add_kd.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

#################################################################################

robot.run(10000, 1./60.)
