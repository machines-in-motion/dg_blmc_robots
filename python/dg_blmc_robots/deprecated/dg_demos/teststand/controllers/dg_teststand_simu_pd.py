import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.teststand import get_teststand_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.operator import Stack_of_vector


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

# Setup the control graph to track the desired joint positions.
pd = ControlPD("PDController")
# setup the gains
pd.Kp.value = 2 * (5.,)
pd.Kd.value = 2 * (0.1,)
# define the desired position and set teh desired velocity 0.
pd.desiredposition.value = np.array(q[1:]).T[0].tolist()
pd.desiredvelocity.value = 2 * (0.,)
# plug the desired quantity signals in the pd controller.
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

# plug the ouput of the pd controller to the robot motor torques
plug(pd.control, robot.device.ctrl_joint_torques)

# Run the robot (here the simulator) for 5000 steps (= 5 seconds). The second
# optional parameter specifies how much to sleep every 1/60. steps. This allows
# a plugged visualizer to display the evolving trajectory.
# for i in range(5):
#     # Pull the robot upwards again.
#     robot.reset_state(q, dq)
#
#     robot.run(1000, 1./60.)
#     print("Contact forces:", robot.device.ati_force.value)
#
# raw_input("Press Enter to continue...")
