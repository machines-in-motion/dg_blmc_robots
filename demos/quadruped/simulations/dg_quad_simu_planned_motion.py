## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller

from dynamic_graph.sot.core.reader import Reader

import time
import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

###### robot init #######################################################
# Get the robot corresponding to the quadruped.

import pinocchio as se3
from pinocchio.utils import zero

robot = get_quadruped_robot(record_video = False)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.22
q[6] = 1.
q[7] = 0.8
q[8] = -1.6
q[9] = 0.8
q[10] = -1.6
q[11] = -0.8
q[12] = 1.6
q[13] = -0.8
q[14] = 1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

###############################################################################

### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_forces = Reader('forces')

filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_velocities_eff.dat"
# filename_forces = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_forces.dat"

# filename_pos = "../trajectories/quadruped_positions_eff_rearing.dat"
# filename_vel = "../trajectories/quadruped_velocities_eff_rearing.dat"


file_exists(filename_pos)
file_exists(filename_vel)


print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'

des_pos = reader_pos.vector
des_vel = reader_vel.vector


###############################################################################

kp = constVector([150.0, 0.0, 150.0, 0.0, 0.0, 0.0], "kp_split")

kd = constVector([2.0, 0.0,2.0, 0.0, 0.0, 0.0], "kd_split")

add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)


##############################################################################

robot.run(5000, 1./60.)
