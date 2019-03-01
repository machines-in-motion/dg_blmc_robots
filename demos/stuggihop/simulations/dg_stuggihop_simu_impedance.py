import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.stuggihop import get_stuggihop_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.operator import Stack_of_vector

from py_dg_blmc_robots.stuggihop import get_stuggihop_robot
from py_dg_blmc_robots.stuggihop import StuggihopConfig
import dynamic_graph.sot.dynamics_pinocchio as dyn_pin

from leg_impedance_control.utils import (hom2pos, stack_zero, constVector, 
    mul_double_vec, mul_mat_vec, compute_pos_diff, stack_two_vectors, 
    Mat_transpose, add_vec_vec, compute_impedance_torques, impedance_controller)

from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.core.operator import (Multiply_double_vector, 
    Add_of_double)


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

######## DEFS FOR IMPEDANCE ########
######## NOW MOVED TO LEG_IMPEDANCE/UTILS ########
# def compute_impedance_torques(jac, errors_vec, names = ["","","",""]):
#     """
#     TODO: add a selector matrix. Currently the selection is not generic at all.
#     """
#     if len(names) != 4:
#         print("ERROR: 4 unique names are required for entities. " 
#                +"You can also leave this empty, and it will generate names.")
#     if names[0] == "":
#         print("WARNING: it is recommended to name every entity yoursef.")

#     ##Transpose tau = JacT*(errors)
#     ## errors = [x - xdes, y-ydes]T
#     jac_T = Mat_transpose(jac, names[0]) # TODO: Is this safe?
#     ## multiplying negative
#     errors_vec = mul_double_vec(-1.0, errors_vec, names[1])
#     control_torques = mul_mat_vec(jac_T, errors_vec, 
#                                         names[2])
#     select_mat = Selec_of_vector(names[3])
#     select_mat.selec(2,4) # This is not generic... should have a selector matrix
#     plug(control_torques, select_mat.signal("sin"))
#     return select_mat.signal("sout")

# def impedance_controller(robot_dg, kv, des_pos, kd = None, des_vel = None,
#     ent_append = "_leg"):
#     ## Impedance control implementation
#     xyzpos_hip = hom2pos(robot_dg.pos_hip, "xyzpos_hip"+ent_append)
#     xyzpos_foot = hom2pos(robot_dg.pos_foot, "xyzpos_foot"+ent_append)
#     # relative foot position to hip
#     rel_pos_foot = compute_pos_diff(xyzpos_foot, xyzpos_hip, 
#         "rel_pos_foot"+ent_append)

#     jac = robot_dg.jac_contact
#     pos_error = compute_pos_diff(rel_pos_foot, des_pos, "pos_error"+ent_append)
#     # Stacking rotations after Cartesian pos_error, to make this into 
#     # an SE3 pos,  since the Jacobian has been taken w.r.t. this (pos, rot)
#     pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0,0.0],
#                                 "stack_to_se3_p"+ent_append), 3, 3)

#     mul_double_vec_op = Multiply_double_vector("gain_multiplication"+ent_append)
#     plug(kv, mul_double_vec_op.sin1)
#     plug(pos_error, mul_double_vec_op.sin2)
#     virtual_spring_force = mul_double_vec_op.sout
#     if kd == None:
#         print("Only spring")
#         control_torques = compute_impedance_torques(jac, virtual_spring_force,
#         [s+ent_append for s in ["jacTranspose","neg_op","comp_ctrl_torques",
#         "impedance_torques"]])
#     else:
#         print("Spring and damper")
#         # also compute virtual damping
#         rel_vel_foot = mul_mat_vec(jac, robot_dg.velocity, 
#             "rel_vel_foot"+ent_append)
#         vel_error = compute_pos_diff(rel_vel_foot,  stack_two_vectors(des_vel, 
#             constVector([0.0, 0.0, 0.0], "stack_to_se3_v"+ent_append), 3, 3), 
#         "vel_error"+ent_append)
#         mul_double_vec_op2 = Multiply_double_vector(
#             "gain_multiplication_vel"+ent_append)
#         print("About to plug kd")
#         plug(kd, mul_double_vec_op2.sin1)
#         print(kd)
#         plug(vel_error, mul_double_vec_op2.sin2)
#         virtual_damper_force = mul_double_vec_op2.sout

#         ### virtual-force =kv*(pos_error) + Kd*(vel_error)
#         virtual_force = add_vec_vec(virtual_spring_force, virtual_damper_force, 
#             "virtual_force"+ent_append)
#         control_torques = compute_impedance_torques(jac, virtual_force,
#         [s+ent_append for s in ["jacTranspose","neg_op","comp_ctrl_torques",
#         "impedance_torques"]])

#     return control_torques

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