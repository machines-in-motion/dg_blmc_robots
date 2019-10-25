import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import dg_blmc_robots
from dg_blmc_robots.stuggihop import get_stuggihop_robot

from dynamic_graph import plug
# from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph_manager.dg_tools import ControlPD
from dynamic_graph.sot.core.operator import Stack_of_vector


# Get the robot corresponding to the robot.
robot = get_stuggihop_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.4
q[2] = 0.8
q[3] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

# Setup the control graph to track the desired joint positions.
pd = ControlPD("controller_1")
# setup the gains
pd.Kp.value = 2 * (5.,)
pd.Kd.value = 2 * (0.1,)
# define the desired position and set teh desired velocity 0.
pd.desired_position.value = (np.pi/4.0, -np.pi/4)
pd.desired_velocity.value = 2 * (0.,)
# plug the desired quantity signals in the pd controller.
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

# plug the ouput of the pd controller to the robot motor torques
plug(pd.control, robot.device.ctrl_joint_torques)

robot.run(50000, 1./60.)

raw_input("Press Enter to continue...")