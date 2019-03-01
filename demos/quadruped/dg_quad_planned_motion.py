## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
import time
from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader

#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

#############################################################################
reader_pos = Reader('PositionReader_squat')
#filename_pos = os.path.abspath('./quadruped/trajectories/quadruped_jumping_positions_eff.dat')
#filename_pos = os.path.abspath('./quadruped/trajectories/quadruped_squatting_positions_eff.dat')
filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"
file_exists(filename_pos)

print("Loading data files:")
reader_pos.load(filename_pos)
#reader_vel.load(filename_vel)
# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '1111111111110'
#reader_vel.selec.value = '1111111111110'

###############################################################################

#pos_des = constVector([0.0, 0.0, -0.25], "pos_des")


##For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 10.0
gain_value = add.sout

def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)

init_pos = reader_pos.vector
pos_des_hl = selec_vector(init_pos, 0, 3, "HL_init")
pos_des_hr = selec_vector(init_pos, 3, 6, "HR_init")
pos_des_fl = selec_vector(init_pos, 6, 9, "FL_init")
pos_des_fr = selec_vector(init_pos, 9,12, "FR_init")

control_torques = quad_impedance_controller(robot, pos_des_fl, pos_des_fr, pos_des_hl, pos_des_hr, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)


####################################### data logging ########################################


# robot.add_trace("rel_pos_foot_fr", "sout")
# robot.add_ros_and_trace("rel_pos_foot_fr", "sout")
#
# robot.add_trace("rel_pos_foot_fl", "sout")
# robot.add_ros_and_trace("rel_pos_foot_fl", "sout")
#
# robot.add_trace("rel_pos_foot_hr", "sout")
# robot.add_ros_and_trace("rel_pos_foot_hr", "sout")
#
# robot.add_trace("rel_pos_foot_hl", "sout")
# robot.add_ros_and_trace("rel_pos_foot_hl", "sout")
