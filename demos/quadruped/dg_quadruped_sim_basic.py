import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.operator import Stack_of_vector


# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

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

# Update the initial state of the robot.
robot.reset_state(q, dq)

# Setup the control graph to track the desired joint positions.
pd = ControlPD("PDController")
# setup the gains
pd.Kp.value = 8 * (5.,)
pd.Kd.value = 8 * (0.1,)
# define the desired position and set teh desired velocity 0.
pd.desiredposition.value = np.array(q[7:]).T[0].tolist()
pd.desiredvelocity.value = 8 * (0.,)
# plug the desired quantity signals in the pd controller.
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)
# plug the ouput of the pd controller to the robot motor torques
plug(pd.control, robot.device.ctrl_joint_torques)

# acquire the base from the robot
base_pose_signal, base_velocity_signal = robot.base_signals()
# reconstruct the joint state
robot_joint_states = Stack_of_vector("robot_joint_states")
robot_joint_states.selec1(0, 7)
robot_joint_states.selec2(0, 8)
# get the base pose
plug(base_pose_signal, robot_joint_states.sin1)
# here we plug the encoders to sin2.
plug(robot.device.joint_positions, robot_joint_states.sin2)

robot.add_robot_state_to_ros(entity_name=robot_joint_states.name,
                             signal_name="sout",
                             base_link_name=robot.base_link_name,
                             joint_names=robot.joint_names,
                             tf_prefix="quadruped",
                             joint_state_topic_name="/quadruped_joint_states")

# Run the robot (here the simulator) for 5000 steps (= 5 seconds). The second
# optional parameter specifies how much to sleep every 1/60. steps. This allows
# a plugged visualizer to display the evolving trajectory.
robot.run(5000, 1./60.)

raw_input("Press Enter to continue...")
