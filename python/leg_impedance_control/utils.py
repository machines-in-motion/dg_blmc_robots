# Impedace functions for quadruped
## based on implementation on teststand
# Author : Avadesh Meduri
# Date : 29th Jan 2019

import os, os.path

import numpy as np
import rospkg

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero

from dynamic_graph import plug
from dynamic_graph.sot.core import *
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.operator import *
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from py_robot_properties_teststand.config import TeststandConfig


def add_vec_vec(vec1, vec2, entityName):
    add = Add_of_vector(entityName)
    plug(vec1, add.signal('sin1'))
    plug(vec2, add.signal('sin2'))
    return add.sout

def compute_pos_diff(pos1, pos2, entityName):
    sub_op = Substract_of_vector(entityName)
    plug(pos1, sub_op.signal('sin1'))
    plug(pos2, sub_op.signal('sin2'))
    return sub_op.sout

def Mat_transpose(mat, entityName):
    op = MatrixTranspose(entityName)
    plug(mat, op.sin)
    return op.sout

def mul_mat_vec(mat,vec, entityName):
    mat_mul = Multiply_matrix_vector(entityName)
    plug(mat, mat_mul.signal('sin1'))
    plug(vec, mat_mul.signal('sin2'))
    return mat_mul.sout

def mul_double_vec(doub, vec, entityName):
    mul = Multiply_double_vector(entityName)
    mul.sin1.value = doub
    plug(vec, mul.signal('sin2'))
    return mul.sout

def stack_two_vectors(vec1, vec2, vec1_size, vec2_size):
    op = Stack_of_vector("")
    op.selec1(0, vec1_size)
    op.selec2(0, vec2_size)
    plug(vec1, op.signal('sin1'))
    plug(vec2, op.signal('sin2'))
    return op.signal('sout')

def stack_zero(sig, entityName):
    zero = VectorConstant(entityName)
    zero.sout.value = (0.,)

    op = Stack_of_vector("")
    op.selec1(0, 1)
    op.selec2(0, 2)
    plug(zero.sout, op.sin1)
    plug(sig, op.sin2)
    return op.sout

def compute_control_torques_fl(jac_fl,errors_fl):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT_fl = Mat_transpose(jac_fl, "jacTranspose_fl")
    ## multiplying negative
    errors_fl = mul_double_vec(-1.0, errors_fl, "neg_op_fl")
    control_torques_fl = mul_mat_vec(jacT_fl,errors_fl, "compute_control_torques_fl")

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel_fl = Selec_of_vector("impedance_torques_fl")
    sel_fl.selec(1,3)
    plug(control_torques_fl, sel_fl.signal('sin'))
    return sel_fl.signal('sout')

def compute_control_torques_fr(jac_fr,errors_fr):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT_fr = Mat_transpose(jac_fr, "jacTranspose_fr")
    ## multiplying negative
    errors_fr = mul_double_vec(-1.0, errors_fr, "neg_op_fr")
    control_torques_fr = mul_mat_vec(jacT_fr,errors_fr, "compute_control_torques_fr")

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel_fr = Selec_of_vector("impedance_torques_fr")
    sel_fr.selec(1,3)
    plug(control_torques_fr, sel_fr.signal('sin'))
    return sel_fr.signal('sout')

def compute_control_torques_hl(jac_hl,errors_hl):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT_hl = Mat_transpose(jac_hl, "jacTranspose_hl")
    ## multiplying negative
    errors_hl = mul_double_vec(-1.0, errors_hl, "neg_op_hl")
    control_torques_hl = mul_mat_vec(jacT_hl,errors_hl, "compute_control_torques_hl")

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel_hl = Selec_of_vector("impedance_torques_hl")
    sel_hl.selec(1,3)
    plug(control_torques_hl, sel_hl.signal('sin'))
    return sel_hl.signal('sout')

def compute_control_torques_hr(jac_hr,errors_hr):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT_hr = Mat_transpose(jac_hr, "jacTranspose_hr")
    ## multiplying negative
    errors_hr = mul_double_vec(-1.0, errors_hr, "neg_op_hr")
    control_torques_hr = mul_mat_vec(jacT_hr,errors_hr, "compute_control_torques_hr")

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel_hr = Selec_of_vector("impedance_torques_hr")
    sel_hr.selec(1,3)
    plug(control_torques_hr, sel_hr.signal('sin'))
    return sel_hr.signal('sout')


def ele_mat_mul(mat1, mat2, entityName):
    ## elements wise multiplication
    op = Multiply_double_vector(entityName)
    plug(mat1, op.sin1)
    plug(mat2, op.sin2)
    return op.sout

def constVector(val, entityName):
    op = VectorConstant(entityName).sout
    op.value = list(val)
    return op

def matrixConstant(val):
    op = MatrixConstant("").sout
    op.value = val
    return op

def hom2pos(sig, entityName):
    conv_pos = MatrixHomoToPose(entityName)
    plug(sig, conv_pos.signal('sin'))
    return conv_pos.signal('sout')

def remove_base_pos(vec1, vec2):
    op = Stack_of_vector("clean")
    op.selec1(0, 1)
    op.selec2(2, 3)
    plug(vec1, op.signal('sin1'))
    plug(vec2, op.signal('sin2'))
    return op.signal('sout')

def selec_vector(vec, start_index, end_index, entityName):
    ## to slice the vector
    op = Selec_of_vector(entityName)
    op.selec(start_index, end_index)
    plug(vec, op.signal('sin'))
    return op.sout

def impedance_controller_fl(robot_fl_dg, gain_value, des_pos):
    ## Impdance control implementation
    xyzpos_hip_fl = hom2pos(robot_fl_dg.pos_hip_fl, "xyzpos_hip_fl")
    xyzpos_foot_fl = hom2pos(robot_fl_dg.pos_foot_fl, "xyzpos_foot_fl")
    # relative foot position to hip
    rel_pos_foot_fl = compute_pos_diff(xyzpos_foot_fl, xyzpos_hip_fl, "rel_pos_foot_fl")

    jac_fl = robot_fl_dg.jac_contact_fl

    pos_error_fl = compute_pos_diff(rel_pos_foot_fl, des_pos, "pos_error_fl")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error_fl = stack_two_vectors(pos_error_fl, constVector([0.0, 0.0, 0.0],'stack_to_wrench_fl'), 3, 3)

    mul_double_vec_op_fl = Multiply_double_vector("gain_multiplication_fl")
    plug(gain_value, mul_double_vec_op_fl.sin1)
    plug(pos_error_fl, mul_double_vec_op_fl.sin2)
    pos_error_with_gains_fl = mul_double_vec_op_fl.sout
    control_torques_fl = compute_control_torques_fl(jac_fl, pos_error_with_gains_fl)

    return control_torques_fl

def impedance_controller_fr(robot_fr_dg, gain_value,des_pos):
    ## Impdance control implementation
    xyzpos_hip_fr = hom2pos(robot_fr_dg.pos_hip_fr, "xyzpos_hip_fr")
    xyzpos_foot_fr = hom2pos(robot_fr_dg.pos_foot_fr, "xyzpos_foot_fr")
    # relative foot position to hip
    rel_pos_foot_fr = compute_pos_diff(xyzpos_foot_fr, xyzpos_hip_fr, "rel_pos_foot_fr")
    ## removing the values of the base
    ##rel_pos_foot = remove_base_pos(rel_pos_foot,rel_pos_foot)

    jac_fr = robot_fr_dg.jac_contact_fr
    pos_error_fr = compute_pos_diff(rel_pos_foot_fr, des_pos, "pos_error_fr")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error_fr = stack_two_vectors(pos_error_fr, constVector([0.0, 0.0, 0.0],'stack_to_wrench_fr'), 3, 3)

    mul_double_vec_op_fr = Multiply_double_vector("gain_multiplication_fr")
    plug(gain_value, mul_double_vec_op_fr.sin1)
    plug(pos_error_fr, mul_double_vec_op_fr.sin2)
    pos_error_with_gains_fr = mul_double_vec_op_fr.sout
    control_torques_fr = compute_control_torques_fr(jac_fr, pos_error_with_gains_fr)

    return control_torques_fr

def impedance_controller_hl(robot_hl_dg, gain_value,des_pos):
    ## Impdance control implementation
    xyzpos_hip_hl = hom2pos(robot_hl_dg.pos_hip_hl, "xyzpos_hip_hl")
    xyzpos_foot_hl = hom2pos(robot_hl_dg.pos_foot_hl, "xyzpos_foot_hl")
    # relative foot position to hip
    rel_pos_foot_hl = compute_pos_diff(xyzpos_foot_hl, xyzpos_hip_hl, "rel_pos_foot_hl")
    ## removing the values of the base
    ##rel_pos_foot = remove_base_pos(rel_pos_foot,rel_pos_foot)

    jac_hl = robot_hl_dg.jac_contact_hl
    pos_error_hl = compute_pos_diff(rel_pos_foot_hl, des_pos, "pos_error_hl")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error_hl = stack_two_vectors(pos_error_hl, constVector([0.0, 0.0, 0.0],'stack_to_wrench_hl'), 3, 3)

    mul_double_vec_op_hl = Multiply_double_vector("gain_multiplication_hl")
    plug(gain_value, mul_double_vec_op_hl.sin1)
    plug(pos_error_hl, mul_double_vec_op_hl.sin2)
    pos_error_with_gains_hl = mul_double_vec_op_hl.sout
    control_torques_hl = compute_control_torques_hl(jac_hl, pos_error_with_gains_hl)

    return control_torques_hl


def impedance_controller_hr(robot_hr_dg, gain_value,des_pos):
    ## Impdance control implementation
    xyzpos_hip_hr = hom2pos(robot_hr_dg.pos_hip_hr, "xyzpos_hip_hr")
    xyzpos_foot_hr = hom2pos(robot_hr_dg.pos_foot_hr, "xyzpos_foot_hr")
    # relative foot position to hip
    rel_pos_foot_hr = compute_pos_diff(xyzpos_foot_hr, xyzpos_hip_hr, "rel_pos_foot_hr")
    ## removing the values of the base
    ##rel_pos_foot = remove_base_pos(rel_pos_foot,rel_pos_foot)

    jac_hr = robot_hr_dg.jac_contact_hr
    pos_error_hr = compute_pos_diff(rel_pos_foot_hr, des_pos, "pos_error_hr")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error_hr = stack_two_vectors(pos_error_hr, constVector([0.0, 0.0, 0.0],'stack_to_wrench_hr'), 3, 3)

    mul_double_vec_op_hr = Multiply_double_vector("gain_multiplication_hr")
    plug(gain_value, mul_double_vec_op_hr.sin1)
    plug(pos_error_hr, mul_double_vec_op_hr.sin2)
    pos_error_with_gains_hr = mul_double_vec_op_hr.sout
    control_torques_hr = compute_control_torques_hr(jac_hr, pos_error_with_gains_hr)

    return control_torques_hr


#########i

def createLegs():

    robot_fl_py = TeststandConfig.buildRobotWrapper()
    robot_fl_dg = dp.DynamicPinocchio('hopper_fl')
    robot_fl_dg.setData(robot_fl_py.data)
    robot_fl_dg.setModel(robot_fl_py.model)
    robot_fl_dg.createJacobianEndEffWorld('jac_contact_fl', 'contact')
    robot_fl_dg.createPosition('pos_hip_fl', 'HFE')
    robot_fl_dg.createPosition('pos_foot_fl', 'END')
    robot_fl_dg.createVelocity('vel_foot_fl', 'END')

    robot_fr_py = TeststandConfig.buildRobotWrapper()
    robot_fr_dg = dp.DynamicPinocchio('hopper_fr')
    robot_fr_dg.setData(robot_fr_py.data)
    robot_fr_dg.setModel(robot_fr_py.model)
    robot_fr_dg.createJacobianEndEffWorld('jac_contact_fr', 'contact')
    robot_fr_dg.createPosition('pos_hip_fr', 'HFE')
    robot_fr_dg.createPosition('pos_foot_fr', 'END')
    robot_fr_dg.createVelocity('vel_foot_fl', 'END')

    robot_hl_py = TeststandConfig.buildRobotWrapper()
    robot_hl_dg = dp.DynamicPinocchio('hopper_hl')
    robot_hl_dg.setData(robot_hl_py.data)
    robot_hl_dg.setModel(robot_hl_py.model)
    robot_hl_dg.createJacobianEndEffWorld('jac_contact_hl', 'contact')
    robot_hl_dg.createPosition('pos_hip_hl', 'HFE')
    robot_hl_dg.createPosition('pos_foot_hl', 'END')
    robot_fl_dg.createVelocity('vel_foot_fl', 'END')

    robot_hr_py = TeststandConfig.buildRobotWrapper()
    robot_hr_dg = dp.DynamicPinocchio('hopper_hr')
    robot_hr_dg.setData(robot_hr_py.data)
    robot_hr_dg.setModel(robot_hr_py.model)
    robot_hr_dg.createJacobianEndEffWorld('jac_contact_hr', 'contact')
    robot_hr_dg.createPosition('pos_hip_hr', 'HFE')
    robot_hr_dg.createPosition('pos_foot_hr', 'END')
    robot_hr_dg.createVelocity('vel_foot_fl', 'END')

    return robot_fl_py, robot_fl_dg, robot_fr_py, robot_fr_dg, robot_hl_py, robot_hl_dg, robot_hr_py, robot_hr_dg
