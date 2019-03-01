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
# from dynamic_graph.sot.core import *
from dynamic_graph.sot.core import Selec_of_vector
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

def compute_impedance_torques(jac, errors_vec, names = ["","","",""]):
    """
    TODO: add a selector matrix. Currently the selection is not generic at all.
    """
    if len(names) != 4:
        print("ERROR: 4 unique names are required for entities. "
               +"You can also leave this empty, and it will generate names.")
    if names[0] == "":
        print("WARNING: it is recommended to name every entity yoursef.")

    ##Transpose tau = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jac_T = Mat_transpose(jac, names[0]) # TODO: Is this safe?
    ## multiplying negative
    errors_vec = mul_double_vec(-1.0, errors_vec, names[1])
    control_torques = mul_mat_vec(jac_T, errors_vec,
                                        names[2])
    select_mat = Selec_of_vector(names[3])
    select_mat.selec(2,4) # This is not generic... should have a selector matrix
    plug(control_torques, select_mat.signal("sin"))
    return select_mat.signal("sout")

def impedance_controller(robot_dg, kv, des_pos, kd = None, des_vel = None,
    ent_append = "_leg"):
    ## Impedance control implementation
    xyzpos_hip = hom2pos(robot_dg.pos_hip, "xyzpos_hip"+ent_append)
    xyzpos_foot = hom2pos(robot_dg.pos_foot, "xyzpos_foot"+ent_append)
    # relative foot position to hip
    rel_pos_foot = compute_pos_diff(xyzpos_foot, xyzpos_hip,
        "rel_pos_foot"+ent_append)

    jac = robot_dg.jac_contact
    pos_error = compute_pos_diff(rel_pos_foot, des_pos, "pos_error"+ent_append)
    # Stacking rotations after Cartesian pos_error, to make this into
    # an SE3 pos,  since the Jacobian has been taken w.r.t. this (pos, rot)
    pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0,0.0],
                                "stack_to_se3_p"+ent_append), 3, 3)

    mul_double_vec_op = Multiply_double_vector("gain_multiplication"+ent_append)
    plug(kv, mul_double_vec_op.sin1)
    plug(pos_error, mul_double_vec_op.sin2)
    virtual_spring_force = mul_double_vec_op.sout
    if kd == None:
        control_torques = compute_impedance_torques(jac, virtual_spring_force,
        [s+ent_append for s in ["jacTranspose","neg_op","comp_ctrl_torques",
        "impedance_torques"]])
    else:
        # also compute virtual damping
        rel_vel_foot = mul_mat_vec(jac, robot_dg.velocity,
            "rel_vel_foot"+ent_append)
        vel_error = compute_pos_diff(rel_vel_foot,  stack_two_vectors(des_vel,
            constVector([0.0, 0.0, 0.0], "stack_to_se3_v"+ent_append), 3, 3),
        "vel_error"+ent_append)
        mul_double_vec_op2 = Multiply_double_vector(
            "gain_multiplication_vel"+ent_append)
        plug(kd, mul_double_vec_op2.sin1)
        plug(vel_error, mul_double_vec_op2.sin2)
        virtual_damper_force = mul_double_vec_op2.sout

        ### virtual-force =kv*(pos_error) + Kd*(vel_error)
        virtual_force = add_vec_vec(virtual_spring_force, virtual_damper_force,
            "virtual_force"+ent_append)
        control_torques = compute_impedance_torques(jac, virtual_force,
        [s+ent_append for s in ["jacTranspose","neg_op","comp_ctrl_torques",
        "impedance_torques"]])

    return control_torques

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
    control_torques_fl = compute_impedance_torques(jac_fl, pos_error_with_gains_fl,
        ["jacTranspose_fl","neg_op_fl","compute_control_torques_fl","impedance_torques_fl"])

    return control_torques_fl

def impedance_controller_fr(robot_fr_dg, Kp, des_pos):
    ## Impdance control implementation
    xyzpos_hip_fr = hom2pos(robot_fr_dg.pos_hip_fr, "xyzpos_hip_fr")
    xyzpos_foot_fr = hom2pos(robot_fr_dg.pos_foot_fr, "xyzpos_foot_fr")
    # relative foot position to hip
    rel_pos_foot_fr = compute_pos_diff(xyzpos_foot_fr, xyzpos_hip_fr, "rel_pos_foot_fr")

    jac_fr = robot_fr_dg.jac_contact_fr
    pos_error_fr = compute_pos_diff(rel_pos_foot_fr, des_pos, "pos_error_fr")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error_fr = stack_two_vectors(pos_error_fr, constVector([0.0, 0.0, 0.0],'stack_to_wrench_fr'), 3, 3)

    mul_double_vec_op_fr = Multiply_double_vector("gain_multiplication_fr")
    plug(Kp, mul_double_vec_op_fr.sin1)
    plug(pos_error_fr, mul_double_vec_op_fr.sin2)
    pos_error_with_gains_fr = mul_double_vec_op_fr.sout

    control_torques_fr = compute_impedance_torques(jac_fr, pos_error_with_gains_fr,
        ["jacTranspose_fr","neg_op_fr","compute_control_torques_fr","impedance_torques_fr"])

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
    control_torques_hl = compute_impedance_torques(jac_hl, pos_error_with_gains_hl,
        ["jacTranspose_hl","neg_op_hl","compute_control_torques_hl","impedance_torques_hl"])

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
    control_torques_hr = compute_impedance_torques(jac_hr, pos_error_with_gains_hr,
        ["jacTranspose_hr","neg_op_hr","compute_control_torques_hr","impedance_torques_hr"])

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


    robot_fr_py = TeststandConfig.buildRobotWrapper()
    robot_fr_dg = dp.DynamicPinocchio('hopper_fr')
    robot_fr_dg.setData(robot_fr_py.data)
    robot_fr_dg.setModel(robot_fr_py.model)
    robot_fr_dg.createJacobianEndEffWorld('jac_contact_fr', 'contact')
    robot_fr_dg.createPosition('pos_hip_fr', 'HFE')
    robot_fr_dg.createPosition('pos_foot_fr', 'END')


    robot_hl_py = TeststandConfig.buildRobotWrapper()
    robot_hl_dg = dp.DynamicPinocchio('hopper_hl')
    robot_hl_dg.setData(robot_hl_py.data)
    robot_hl_dg.setModel(robot_hl_py.model)
    robot_hl_dg.createJacobianEndEffWorld('jac_contact_hl', 'contact')
    robot_hl_dg.createPosition('pos_hip_hl', 'HFE')
    robot_hl_dg.createPosition('pos_foot_hl', 'END')


    robot_hr_py = TeststandConfig.buildRobotWrapper()
    robot_hr_dg = dp.DynamicPinocchio('hopper_hr')
    robot_hr_dg.setData(robot_hr_py.data)
    robot_hr_dg.setModel(robot_hr_py.model)
    robot_hr_dg.createJacobianEndEffWorld('jac_contact_hr', 'contact')
    robot_hr_dg.createPosition('pos_hip_hr', 'HFE')
    robot_hr_dg.createPosition('pos_foot_hr', 'END')


    return robot_fl_py, robot_fl_dg, robot_fr_py, robot_fr_dg, robot_hl_py, robot_hl_dg, robot_hr_py, robot_hr_dg
