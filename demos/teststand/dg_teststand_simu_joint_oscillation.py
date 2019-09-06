## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 12/02/19



import pinocchio as se3
from pinocchio.utils import zero

from dg_teststand_joint_oscillation import JointOscillation

from py_dg_blmc_robots.teststand import get_teststand_robot

# Get the robot corresponding to the quadruped.
robot = get_teststand_robot(fixed_slider=False, slider_a_init_value=0.0, slider_b_init_value=0.0)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

###################################################################################

joint_oscillator = JointOscillation(robot)

#########################################################################
robot.run(60000, 1./60.)
