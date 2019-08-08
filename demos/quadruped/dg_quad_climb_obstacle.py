## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019

import os
import time
from dynamic_graph import plug
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.switch import SwitchVector
from leg_impedance_control.utils import constVector, Add_of_double
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller

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

def get_controller(robot):
    ###
    class LocalController():
        def __init__(self, name):
            self.name = name
    ctrl = LocalController("climb_obstacle")

    ### reading createData
    traj_root_folder = "/tmp/"
    reader_pos, reader_vel, reader_com = read_planned_traj(traj_root_folder)
    # save the pointer for further online access
    ctrl.reader_pos = reader_pos
    ctrl.reader_vel = reader_vel
    ctrl.reader_com = reader_com
    # shortcuts
    des_pos_traj = ctrl.reader_pos.vector
    des_vel_traj = ctrl.reader_vel.vector

    ### setup the default controller
    # ('com: ', matrix([[0.2       , 0.00044412, 0.19240999]]))
    # ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
    # ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))
    # ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
    # ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
    des_pos_default = constVector([
      0.0, 0.0, -0.22294615, 0.0, 0.0, 0.0,
      0.0, 0.0, -0.22294615, 0.0, 0.0, 0.0,
      0.0, 0.0, -0.22294615, 0.0, 0.0, 0.0,
      0.0, 0.0, -0.22294615, 0.0, 0.0, 0.0,], ctrl.name + "_default_pos")
    des_vel_default = constVector(24*[0.0,], ctrl.name + "_default_vel")

    ### Setup the switches
    control_switch_pos = SwitchVector("control_switch_pos")
    control_switch_pos.setSignalNumber(2) # we want to switch between 2 signals
    plug(des_pos_default, control_switch_pos.sin0)
    plug(des_pos_traj, control_switch_pos.sin1)
    control_switch_pos.selection.value = 0 # pick and switch manually
    ctrl.control_switch_pos = control_switch_pos
    
    control_switch_vel = SwitchVector("control_switch_vel")
    control_switch_vel.setSignalNumber(2) # we want to switch between 2 signals
    plug(des_vel_default, control_switch_vel.sin0)
    plug(des_vel_traj, control_switch_vel.sin1)
    control_switch_vel.selection.value = 0 # pick and switch manually
    ctrl.control_switch_vel = control_switch_vel

    # shortcuts
    des_pos = control_switch_pos.sout
    des_vel = control_switch_vel.sout

    ### Setup the control gains
    kp = constVector([100.0, 0.0, 100.0, 0.0, 0.0, 0.0], ctrl.name + "_kp")
    kd = constVector([0.2, 0.0, 0.2, 0.0, 0.0, 0.0], ctrl.name + "_kd")
    # 
    ctrl.kp = kp
    ctrl.kd = kp

    ### Setup the impedance control
    ctrl.quad_imp_ctrl = quad_leg_impedance_controller(robot)
    ctrl.control_torques = ctrl.quad_imp_ctrl.return_control_torques(
      kp, des_pos, kd, des_vel)

    ### plug the impendance control to the robot
    plug(ctrl.control_torques, robot.device.ctrl_joint_torques)

    return ctrl

def go0(ctrl):
    ctrl.control_switch_pos.selection.value = 0
    ctrl.control_switch_vel.selection.value = 0

def go0_and_reset_motion(ctrl):
    go0(ctrl)
    ctrl.reader_pos.rewind()
    ctrl.reader_vel.rewind()
    ctrl.reader_com.rewind()

def start_motion(ctrl):
    ctrl.control_switch_pos.selection.value = 1
    ctrl.control_switch_vel.selection.value = 1

if 'robot' in globals():
    ctrl = get_controller(robot)