## simple impedance controller implementation on impact test stand
## Author : Elham

import pinocchio as se3
from pinocchio.utils import zero
from dg_teststand_planned_motion import PlannedMotion
from dg_blmc_robots.teststand import get_teststand_robot

# Get the robot corresponding to the quadruped.
robot = get_teststand_robot(fixed_slider=False)

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

###################################################################################

stiffness_meas = PlannedMotion(robot)

#########################################################################


for i in range(10000):
    robot.run(1, 1./60.)
    print(robot.device.ctrl_joint_torques.value)
