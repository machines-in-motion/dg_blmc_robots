## This code is a simulation for the impedance controller

## Author: Avadesh Meduri
## Date: 1/03/2019

import numpy as np
import pinocchio as se3
from pinocchio.utils import zero
from dynamic_graph import plug

from dg_blmc_robots.teststand import get_teststand_robot
from dg_blmc_robots.teststand import TeststandConfig

from dynamic_graph_manager.dg_tools import Calibrator
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph_manager.dg_tools import ControlPD
from dynamic_graph.sot.core.operator import Stack_of_vector
from leg_impedance_control.utils import *

################################################################################

#### Simulation Setup
# Get the robot corresponding to the quadruped.
robot = get_teststand_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)
robot.set_gravity((0.0,0.0,0.0))
#### Calibrator setup

joint_calibrator = Calibrator('joint_calibrator')

## Make signals to feed into calibrator
# will need to match the sign depending on the hardstop positions
# calib_vel = constVector([0.001, 0.001,
#                        0.0, 0.0,
#                        0.0, 0.0,
#                        0.0, 0.0,],
#                         "calib_vel")

# calib_kp = constVector(2*[1.0,],"calib_kp")
# This would usually be measured and fixed
# calib_hard2zero = constVector([0.1, 0.1,
#                        0.0, 0.0,
#                        0.0, 0.0,
#                        0.0, 0.0,],"hard2zero")

## plug inputs in
plug(robot.device.joint_positions, joint_calibrator.rawPosition)
plug(robot.device.joint_velocities, joint_calibrator.velocity)
# plug(calib_vel, joint_calibrator.desiredVelocity)
joint_calibrator.desiredVelocity.value = [0.3, 0] # 2*[0.0,]
joint_calibrator.kp.value = 2*[8.0,]
joint_calibrator.hardstop2zero.value = 2*[0.0,]
# plug(calib_kp, joint_calibrator.kp)
# plug(calib_hard2zero, joint_calibrator.hardstop2zero)

#### PD control setup
# des_pos = constVector(2*[0.0,],
#                         "des_pos")

# des_vel = constVector(2*[0.0,],
#                         "des_vel")

# # Setup the control graph to track the desired joint positions.
# pd = ControlPD("PDController")
# # setup the gains
# pd.Kp.value = 4 * (5.,)
# pd.Kd.value = 4 * (0.1,)
# # define the desired position and set teh desired velocity 0.
# pd.desired_position.value = np.array(q[7:]).T[0].tolist()
# pd.desired_velocity.value = 4 * (0.,)
# # plug
# plug(joint_calibrator.calibratedPosition, pd.position)
# plug(robot.device.joint_velocities, pd.velocity)

#### create a switch for the output torque

control_switch = SwitchVector("control_switch")
control_switch.setSignalNumber(2) # we want to switch between 2 signals
plug(joint_calibrator.control, control_switch.sin0)
# plug(pd.control, control_switch.sin1)
control_switch.sin1.value = [0.0,0.0]
control_switch.selection.value = 0 # pick and switch manually for now

plug(control_switch.sout, robot.device.ctrl_joint_torques)

#################################################################################
# acquire the base from the robot

robot.run(50000, 1./60.)

# quad_imp_ctrl.record_data(record_vicon=False)

# robot.add_trace("pos_des", "sout")
# robot.add_ros_and_trace("pos_des", "sout")

# robot.add_trace("vel_des", "sout")
# robot.add_ros_and_trace("vel_des", "sout")
