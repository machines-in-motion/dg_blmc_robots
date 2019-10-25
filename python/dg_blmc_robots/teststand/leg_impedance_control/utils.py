## Contains generic impedance control functions that will be used by the impedance_controller
## Author: Avadesh Meduri
## Date : 1st March 2019


#################### Imports #################################################

import os, os.path

import numpy as np
import rospkg


from dynamic_graph import plug
from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.core.operator import *
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.matrix_constant import MatrixConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

##############################################################################

################### Initialisers #############################################

def constVector(val, entityName=''):
    """
    ## This function initialises an constant vector
    ## Input : array (python list)
    """
    op = VectorConstant(entityName).sout
    op.value = list(val)
    return op

def constMatrix(val, entityName):
    """
    ## This function initialises an constant matrix
    ## Input : matrix (python array)
    """
    op = MatrixConstant(entityName).sout
    op.value = val
    return op



    #################### Operators ################################################

def stack_two_vectors(vec1, vec2, vec1_size, vec2_size):
    """
    ## This function stacks two vectors
    ## Input : Constant vector (not numpy arrays)
          : Constant vector (not numpy arrays)
          : size of first vector (int)
          : size of first vector (int)
    """
    op = Stack_of_vector("")
    op.selec1(0, vec1_size)
    op.selec2(0, vec2_size)
    plug(vec1, op.signal('sin1'))
    plug(vec2, op.signal('sin2'))
    return op.signal('sout')

def stack_zero(vec, entityName=''):
    """
    ## This function stacks a zeros before the vector
    ## Input : Constant vector (not numpy arrays)
          : Constant vector (not numpy arrays)
          : size of first vector (int)
          : size of first vector (int)
    """
    zero = VectorConstant("zero")
    zero.sout.value = (0.,)

    op = Stack_of_vector(entityName)
    op.selec1(0, 1)
    op.selec2(0, 2)
    plug(zero.sout, op.sin1)
    plug(vec, op.sin2)
    return op.sout

def selec_vector(vec, start_index, end_index, entityName=''):
    """
    ## This function selects a part of the input vector (slices vector)
    ## Input : Constant vector (not numpy array)
         : start index (int)
         : end index (int)
    ## Ex    : selec_vector([1,2,3,4], 1,3) = [2,3] {input must be a const vector}
    """
    op = Selec_of_vector(entityName)
    op.selec(start_index, end_index)
    plug(vec, op.signal('sin'))
    return op.sout

def component_of_vector(vector, index, entityName):
    """
    ## This function selects a compnent of the input vector
    ## Input : Constant vector (not numpy array)
         : index (int)
    """
    comp_of_vect = Component_of_vector(entityName)
    comp_of_vect.setIndex(index)
    plug(vector, comp_of_vect.sin)
    return comp_of_vect.sout


    ###################  Math Operators ##########################################

def add_doub_doub(db1, db2, entityName):
    """
    ## This function adds two doubles
    ## Input : db1 - double (number)
             : db2 - double (value)
    """
    add = Add_of_double(entityName)
    add.sin1.value = db1
    add.sin2.value = db2
    return add


def add_vec_vec(vec1, vec2, entityName=''):
    """
    ## This function adds two Vectors
    ## Input : Constant vectors (not numpy arrays)
    """
    add = Add_of_vector(entityName)
    plug(vec1, add.signal('sin1'))
    plug(vec2, add.signal('sin2'))
    return add.sout

def subtract_vec_vec(pos1, pos2, entityName=''):
    """
    ## This function subtracts two Vectors
    ## Input : Constant vectors (not numpy arrays)
    """
    sub_op = Substract_of_vector(entityName)
    plug(pos1, sub_op.signal('sin1'))
    plug(pos2, sub_op.signal('sin2'))
    return sub_op.sout

def transpose_mat(mat, entityName=''):
    """
    ## This function transposes a matrix
    ## Input : Constant matrix (not numpy arrays)
    """
    op = MatrixTranspose(entityName)
    plug(mat, op.sin)
    return op.sout

def multiply_mat_vec(mat,vec, entityName=''):
    """
    ## This function multiplies a matrix and vector
    ## Input : Constant matrix (not numpy arrays)
             : Constant vector (not numpy array)
    """
    mat_mul = Multiply_matrix_vector(entityName)
    plug(mat, mat_mul.signal('sin1'))
    plug(vec, mat_mul.signal('sin2'))
    return mat_mul.sout

def mul_double_vec(doub, vec, entityName=''):
    """
    ## This function multiplies a double and vector
    ## Input : Constant double
             : Constant vector (not numpy array)
    """
    mul = Multiply_double_vector(entityName)
    mul.sin1.value = doub
    plug(vec, mul.signal('sin2'))
    return mul.sout

def mul_vec_vec(vec1, vec2, entityName):
    """
    ## This function multiplies two Vectors element wise
    ## Input : Constant vectors (not numpy arrays)
    """
    vec_mul = Multiply_of_vector(entityName)
    plug(vec1, vec_mul.sin0)
    plug(vec2, vec_mul.sin1)
    return vec_mul.sout


    ######################### Robotics operators ##################################

def hom2pos(robot_joint_signal, entityName=''):
    """
    ## This function transforms a homogenous matrix to xyz cordinate
    ## Input : robot (DynamicPinocchio model) joint signal
    """
    conv_pos = MatrixHomoToPose(entityName)
    plug(robot_joint_signal, conv_pos.signal('sin'))
    return conv_pos.signal('sout')

def convert_quat_se3(quat, entityName):
    """
    ## This function transforms a quaternion(4d) to se3
    ## Input : quaternion signal
    """

    quat_to_se3 = QuaternionToMatrix(entityName)
    plug(quat, quat_to_se3.signal('sin'))
    return quat_to_se3.signal('sout')



    ######################### Standard vectors ####################################

def zero_vec(vec_size, entityName):
    """
    ## This function creates a zero constvector of vec_size
    ## Input : size of zero vector (int)
    """
    zero_vec = np.zeros(vec_size)
    return constVector(zero_vec, entityName)
