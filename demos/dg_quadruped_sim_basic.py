import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD


# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[1] = 0.1
q[2] = 0.4
q[6] = 1.
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

# Setup the control graph to track the desired joint positions.
pd = ControlPD("PDController")
pd.Kp.value = 8 * (5.,)
pd.Kd.value = 8 * (0.1,)

pd.desiredposition.value = np.array(q[7:]).T[0].tolist()
pd.desiredvelocity.value = 8 * (0.,)


plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

# Run the robot (here the simulator) for 5000 steps (= 5 seconds). The second
# optional parameter specifies how much to sleep every 1/60. steps. This allows
# a plugged visualizer to display the evolving trajectory.
robot.run(5000, 1./60.)

