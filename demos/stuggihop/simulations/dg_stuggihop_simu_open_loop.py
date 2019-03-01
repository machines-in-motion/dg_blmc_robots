import numpy as np

# import pinocchio as se3z
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.stuggihop import get_stuggihop_robot

from dynamic_graph import plug

from py_dg_blmc_robots.stuggihop import get_stuggihop_robot
from py_dg_blmc_robots.stuggihop import StuggihopConfig
import dynamic_graph.sot.dynamics_pinocchio as dyn_pin

from leg_impedance_control.utils import (stack_two_vectors, constVector,
    compute_impedance_torques, impedance_controller)
from dynamic_graph.sot.core.operator import Add_of_double

######## DEFS 
print("Begin")
robot_py = StuggihopConfig.buildRobotWrapper()
robot_dg = dyn_pin.DynamicPinocchio("hopper")
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld("jac_contact", "contact")
robot_dg.createPosition("pos_hip", "HFE")
robot_dg.createPosition("pos_foot", "END")

# ######## SETUP ########
print("Starting Setup")
# Get the robot corresponding to the stuggi hop.
robot = get_stuggihop_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.25
q[2] = 0.8
q[3] = -1.6

# Update the initial state of the robot.
robot.set_gravity((0,0,0))
robot.reset_state(q, dq)

robot_dg.acceleration.value = 4 * (0.0, )

# Plug the position and velocity from the robot into the robot_dg.
plug(stack_two_vectors(constVector([0.0, 0.0],"add_base_joint_position"), 
    robot.device.signal("joint_positions"), 2, 2), robot_dg.position)
plug(stack_two_vectors(constVector([0.0, 0.0], "add_base_joint_velocities"), 
    robot.device.signal("joint_velocities"), 2, 2), robot_dg.velocity)
print("Setup finished")

######## CONTROL ########

# For making gain input dynamic through terminal
# TODO: use a const instead....
add_kp = Add_of_double("kv")
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 50.0 # virtual stiffness

add_kd = Add_of_double("kd")
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.5 # virtual damping

# in end-effector cartesian frame (x-y-z)
des_pos = constVector([0.0, 0.0, -0.2],"pos_des")

# in end-effector cartesian frame (x-y-z)
des_vel = constVector([0.0, 0.0, 0.0],"pos_vel")

## Impdance control implementation

# control_torques = impedance_controller(robot_dg, add_kp.sout, des_pos)
control_torques = impedance_controller(robot_dg, add_kp.sout, des_pos, add_kd.sout, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

##### RUNNING SIMULATION
robot.run(50000, 1./60.)