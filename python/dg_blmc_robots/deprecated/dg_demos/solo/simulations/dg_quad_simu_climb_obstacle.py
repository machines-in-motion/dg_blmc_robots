## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

import os
import time
import pinocchio as se3
from pinocchio.utils import zero
from dynamic_graph import plug
from dynamic_graph.sot.core.reader import Reader

from leg_impedance_control.utils import constVector, Add_of_double
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller

from dg_blmc_robots.solo.solo_bullet import get_quadruped_robot

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

def read_planned_traj(path):
    reader_pos = Reader('reader_pos')
    reader_vel = Reader('reader_vel')
    reader_com = Reader('reader_com')

    filename_pos = path + "quadruped_positions_eff.dat"
    filename_vel = path + "quadruped_velocities_eff.dat"
    filename_com = path + "quadruped_com.dat"

    file_exists(filename_pos)
    file_exists(filename_vel)
    file_exists(filename_com)

    print("Loading data files:")
    reader_pos.load(filename_pos)
    reader_vel.load(filename_vel)
    reader_com.load(filename_com)

    # Specify which of the columns to select.
    # NOTE: This is selecting the columns in reverse order - the last number is the first column in the file

    reader_pos.selec.value = '111111111111111111111111'
    reader_vel.selec.value = '111111111111111111111111'
    reader_com.selec.value = '1111'

    return reader_pos, reader_vel, reader_com

###############################################################################

### robot init
# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot(record_video = False)

# add an obstacle in the simulation
# import rospkg
# import pybullet as p
# box_urdf = (rospkg.RosPack().get_path("robot_properties_quadruped") +
#                       "/urdf/box.urdf")
# BoxId = p.loadURDF(box_urdf)


# Define the desired position.
q = robot.q0.copy()
dq = robot.dq0.copy()

# init poses
oMb = robot.pin_robot.framePlacement(q, 1)
oMhl = robot.pin_robot.framePlacement(q, robot.hl_index)
oMhr = robot.pin_robot.framePlacement(q, robot.hr_index)
oMfl = robot.pin_robot.framePlacement(q, robot.fl_index)
oMfr = robot.pin_robot.framePlacement(q, robot.fr_index)
bMhl = oMb.inverse() * oMhl
bMhr = oMb.inverse() * oMhr
bMfl = oMb.inverse() * oMfl
bMfr = oMb.inverse() * oMfr
print ("com: ", robot.pin_robot.com(q).T)
print ("hl: ", bMhl.translation.T)
print ("hr: ", bMhr.translation.T)
print ("fl: ", bMfl.translation.T)
print ("fr: ", bMfr.translation.T)
# ('com: ', matrix([[0.2       , 0.00044412, 0.19240999]]))
# ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
# ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
# ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
# ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))

# Update the initial state of the robot.
robot.reset_state(q, dq)

### reading createData
traj_root_folder = "/tmp/"
reader_pos, reader_vel, reader_com = read_planned_traj(traj_root_folder)

des_pos = reader_pos.vector
des_vel = reader_vel.vector


### Setup the control gains
kp = constVector([100.0, 0.0, 100.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([0.2, 0.0, 0.2, 0.0, 0.0, 0.0], "kd_split")

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)


##############################################################################

robot.run(15000, 1./60.)

print(des_pos.value)
print(des_vel.value)
