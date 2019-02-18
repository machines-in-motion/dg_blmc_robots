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

##########################################################################################


###################################################################################
robot_py = TeststandConfig.buildRobotWrapper()

robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'HFE')
robot_dg.createPosition('pos_foot', 'END')

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

    mul_double_vec_op1 = Multiply_double_vector("gain_multiplication_pos")
    plug(Kp, mul_double_vec_op1.sin1)
    plug(pos_error, mul_double_vec_op1.sin2)
    pos_error_with_gains = mul_double_vec_op1.sout

    rel_vel_foot = mul_mat_vec(jac, robot_dg.velocity, "rel_vel_foot")
    vel_error = compute_pos_diff(rel_vel_foot, des_vel, 'vel_error')
    #
    mul_double_vec_op2 = Multiply_double_vector("gain_multiplication_vel")
    plug(Kd, mul_double_vec_op2.sin1)
    plug(vel_error, mul_double_vec_op2.sin2)
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
kp = Add_of_double('Kp')
kp.sin1.value = 0
### Change this value for different gains
kp.sin2.value = 50.0
Kp = kp.sout

kd = Add_of_double('Kd')
kd.sin1.value = 0
### Change this value for different gains
kd.sin2.value = 0.0
Kd = kd.sout


des_pos, des_vel = linear_sine_generator(0.08, 4.0, 0.0 , -.2, "hopper")

control_torques = impedance_controller(robot_dg, Kp, Kd, des_pos, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)

#########################################################################

robot.add_trace("pos_error", "sout")
robot.add_ros_and_trace("pos_error", "sout")

robot.add_trace("des_position", "sout")
robot.add_ros_and_trace("des_position", "sout")

robot.add_trace("rel_pos_foot", "sout")
robot.add_ros_and_trace("rel_pos_foot", "sout")

robot.add_trace("des_velocity", "sout")
robot.add_ros_and_trace("des_velocity", "sout")

robot.add_trace("rel_vel_foot", "sout")
robot.add_ros_and_trace("rel_vel_foot", "sout")
