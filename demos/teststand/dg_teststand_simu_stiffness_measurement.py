## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 12/02/19



import pinocchio as se3
from pinocchio.utils import zero

from dg_teststand_stiffness_measurement import StiffnessMeasurement

from py_dg_blmc_robots.teststand import get_teststand_robot

# Get the robot corresponding to the quadruped.
robot = get_teststand_robot(fixed_slider=False)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

###################################################################################

stiffness_meas = StiffnessMeasurement(robot)

#########################################################################

robot.run(10000, 1./60.)
