## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 12/02/19

import os, os.path

import numpy as np
import rospkg

import pinocchio as se3
from pinocchio.utils import zero
from pinocchio.robot_wrapper import RobotWrapper

from dynamic_graph import plug
from dynamic_graph.sot.core import *
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.operator import MatrixTranspose
from dynamic_graph.sot.core.operator import *
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

############################################################################

from leg_impedance_control.utils import *
from leg_impedance_control.traj_generators import *


###### robot init #######################################################
#
# from leg_impedance_control.utils import *
# from leg_impedance_control.controller import *
# from leg_impedance_control.traj_generators import *
#
# from dynamic_graph_manager.device import Device
# from dynamic_graph_manager.device.robot import Robot
#
#
# local_device = Device("hopper_robot")
# yaml_path = os.path.join(rospkg.RosPack().get_path("dg_blmc_robots"),'demos/config/teststand.yaml')
# local_device.initialize(yaml_path)
# robot = Robot(name=local_device.name, device=local_device)
#


##############################################################################

import py_dg_blmc_robots
from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig


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

###################################################################################
robot_py = TeststandConfig.buildRobotWrapper()

robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'HFE')
robot_dg.createPosition('pos_foot', 'END')
robot_dg.createVelocity('vel_hip', "HFE")
robot_dg.createVelocity('vel_foot', "END")

########################################################################################

##########################################################################################

def compute_control_torques(jac,errors):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT = Mat_transpose(jac, "jacTranspose")
    ## multiplying negative
    errors = mul_double_vec(-1.0, errors, "neg_op")
    control_torques = mul_mat_vec(jacT,errors, "compute_control_torques")

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel = Selec_of_vector("impedance_torques")
    sel.selec(1,3)
    plug(control_torques, sel.signal('sin'))
    return sel.signal('sout')

def impedance_controller(robot_dg, kp, kd ,des_pos, des_vel):
    ## Impdance control implementation
    xyzpos_hip = hom2pos(robot_dg.pos_hip, "xyzpos_hip")
    xyzpos_foot = hom2pos(robot_dg.pos_foot, "xyzpos_foot")
    # relative foot position to hip
    rel_pos_foot = compute_pos_diff(xyzpos_foot, xyzpos_hip, "rel_pos_foot")
    ## removing the values of the base

    jac = robot_dg.jac_contact
    pos_error = compute_pos_diff(rel_pos_foot, des_pos, "pos_error")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0, 0.0],'stack_to_wrench'), 3, 3)

    rel_vel_foot = compute_pos_diff(robot_dg.vel_foot, robot_dg.vel_hip , "rel_vel_foot")
    vel_error = compute_pos_diff(rel_vel_foot, des_vel, 'vel_error')

    mul_double_vec_op1 = Multiply_double_vector("gain_multiplication_pos")
    plug(Kp, mul_double_vec_op1.sin1)
    plug(pos_error, mul_double_vec_op1.sin2)
    pos_error_with_gains = mul_double_vec_op1.sout

    mul_double_vec_op2 = Multiply_double_vector("gain_multiplication_vel")
    plug(Kd, mul_double_vec_op2.sin1)
    plug(pos_error, mul_double_vec_op2.sin2)
    vel_error_with_gains = mul_double_vec_op2.sout

    ### error = Kp*(pos_error) + Kd*(vel_error)
    total_error = add_vec_vec(pos_error_with_gains, vel_error_with_gains, "total_error")

    control_torques = compute_control_torques(jac, total_error)

    return control_torques

robot_dg.acceleration.value = 3 * (0.0, )

########################################################################################################

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), robot_dg.velocity)

# For making gain input dynamic through terminal
kp = Add_of_double('mult')
kp.sin1.value = 0
### Change this value for different gains
kp.sin2.value = 180.0
Kp = kp.sout

kd = Add_of_double('mult')
kd.sin1.value = 0
### Change this value for different gains
kd.sin2.value = 50.0
Kd = kd.sout


# des_pos = constVector([0.0, 0.0, -0.2],"pos_des")
# des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")
## Impdance control implementation


des_pos, des_vel = linear_sine_generator(0.06, 1.0, 0.0 , -.2, "hopper")

control_torques = impedance_controller(robot_dg, Kp, Kd, des_pos, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

#########################################################################

robot.run(10000, 1./60.)


############ Plotting #############################################


# robot.add_trace("pos_error", "sout")
# robot.add_ros_and_trace("pos_error", "sout")
#
# robot.add_trace("rel_pos_foot", "sout")
# robot.add_ros_and_trace("rel_pos_foot", "sout")
######## robot simulation ################################################
#
# from pinocchio.utils import zero
#
# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)
#
# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)
#
# q[:] = np.asmatrix(robot.device.joint_positions.value).T
# dq[:] = np.asmatrix(robot.device.joint_velocities.value).T
#
# print(q, dq)
#
# dt = 0.001#config.dt
# motor_inertia = 0.045
#
# for i in range(4):
#     # fill the sensors
#     robot.device.joint_positions.value = q.T.tolist()[0][:]
#     robot.device.joint_velocities.value = dq.T.tolist()[0][:]
#
#     # "Execute the dynamic graph"
#     robot.device.executeGraph()
#
#
#     joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
#     #joint_torques[:] = control_torques.value
#
#     des_pos.recompute(i)
#     des_vel.recompute(i)
#
#     print(des_pos.value, des_vel.value)
#
#
#     # integrate the configuration from the computed torques
#     #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
#     #dq = (dq + dt * joint_torques / motor_inertia)
#     q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
#     dq = (dq + dt * 0.01 / motor_inertia)
#
#
#     if (i % 1000) == 0:
#         # print "qref =     ", robot.pid_control.pose_controller.qRef.value
#         # print ("q =        ",
#         #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
#         # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
#         # print "currents = ", robot.device.ctrl_joint_torques.value
#         pass
#
#
# print
# print "End of simulation"
# print "control_torques = ", robot.device.ctrl_joint_torques.value
