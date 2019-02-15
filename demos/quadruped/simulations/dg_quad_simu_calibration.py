## This code is a simulation for the impedance controller

## Author: Steve Heim
## Date: 12/02./2019

import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot
from dynamic_graph.sot.core.control_pd import ControlPD

import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

from dynamic_graph import plug
from dynamic_graph.sot.torque_control.nd_trajectory_generator import NdTrajectoryGenerator
from time import sleep

##########################################################################################
# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()
robot.set_gravity((0,0,0))
# TODO: setGravity(0,0, 0) from here
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

# # Update the initial state of the robot.
robot.reset_state(q, dq)

#########################################################################################

# set desired joint positions to current positions
des_pos = np.array(q[7:]).T[0]
des_vel = np.array(8*(0.,))
# set soft PD gains
# set threshold for current
error_thresh = 0.1 # actually, probably want to use an error threshold...
is_calibrated = 8*[False,]


########### START OF STUPID WAY ########

# start traj gen entity going from current position to arbitrary high position
# with very slow trajectory (set T large)
x_0 = np.array(q[7:]).T[0]
pi = np.pi
x_f = np.array(8*[2*np.pi,]);
x_f = np.array([pi,-2*pi,pi,-2*pi,-pi,2*pi,-pi,2*pi])

traj_gen = NdTrajectoryGenerator('traj_gen');
traj_gen.trigger.value = 1;
dt = 0.001 # ...where to get this
traj_gen.init(dt, 8); # 3??
traj_gen.initial_value.value = x_0
traj_gen.x.recompute(0)
T = 5.0 # total time for traj
# jid = 0 # ???

# traj_gen.move(jid, x_f[jid], T)
for jdx in range(8):
    traj_gen.move(jdx, x_f[jdx], T)
#########################################################################################

# Start PD controller entity tracking desired position
# and plug traj gen to PD controller
pd = ControlPD("PDController")
 
pd.Kp.value = 8 * [3.,]
pd.Kd.value = 8 * [0.1,]

plug(traj_gen.x, pd.desiredposition)
plug(traj_gen.dx, pd.desiredvelocity)
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)
plug(pd.control, robot.device.ctrl_joint_torques)

# Move to trajectory

# check if error threshold goes over a certain amount
for jdx in range(8):
    if abs(traj_gen.x.value[jdx] - robot.device.joint_positions.value[jdx]) > error_thresh: # CHECK is this okay?
        traj_gen.stop(jdx) # if we go over current threshold, freeze position
        is_calibrated[jdx] = True
    # unplug that traj gen, and freeze that desired position


if all(is_calibrated)==True:
    sleep(0.1) # let it settle, just in case. Can probably remove
    robot.device.ctrl_joint_torques.value = 8*(0.,)
    sleep(1)
    # TODO: toggle "is_calibrated" in the robot (should turn on using the actual offsets)
    # TODO: This requires some modification of the quadruped class first
    # TODO: and then go to some desired initial condition

robot.run(10000, 1./60.)
# robot.run(time, at freq)
