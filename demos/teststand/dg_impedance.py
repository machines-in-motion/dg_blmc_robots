## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 23 January 2019

import os, os.path

import numpy as np
import rospkg

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper

from dynamic_graph import plug
from dynamic_graph.sot.core import *
import dynamic_graph.sot.dynamics_pinocchio as dp
#from dynamic_graph.sot.core.operator import MatrixTranspose
from dynamic_graph.sot.core.operator import *
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


urdf_path = os.path.join(rospkg.RosPack().get_path("dg_blmc_robots"),'demos/hopper_1d.urdf')
robot_py = se3.RobotWrapper(urdf_path,[os.path.join(rospkg.RosPack().get_path("robot_properties_quadruped"), 'urdf')])
#robot_py = se3.RobotWrapper(urdf_path,[os.path.join(rospkg.RosPack().get_path("robot_properties_quadruped"), 'urdf')])
robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

#robot_dg.createJacobianEndEffWorld('Jac', 'contact')
robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'joint_hip')
robot_dg.createPosition('pos_foot', 'joint_contact')

def compute_pos_diff(pos1, pos2, entityName):
    sub_op = Substract_of_vector(entityName)
    plug(pos1, sub_op.signal('sin1'))
    plug(pos2, sub_op.signal('sin2'))
    return sub_op.sout

def Mat_transpose(mat):
    op = MatrixTranspose("")
    plug(mat, op.sin)
    return op.sout

def mul_mat_vec(mat,vec):
    mat_mul = Multiply_matrix_vector('')
    plug(mat, mat_mul.signal('sin1'))
    plug(vec, mat_mul.signal('sin2'))
    return mat_mul.sout

def mul_double_vec(doub, vec):
    mul = Multiply_double_vector("")
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

def stack_zero(sig):
    zero = VectorConstant("")
    zero.sout.value = (0.,)

    op = Stack_of_vector("")
    op.selec1(0, 1)
    op.selec2(0, 2)
    plug(zero.sout, op.sin1)
    plug(sig, op.sin2)
    return op.sout

def compute_control_torques(jac,errors):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jacT = Mat_transpose(jac)
    ## multiplying negative
    errors = mul_double_vec(-1.0, errors)
    control_torques = mul_mat_vec(jacT,errors)

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel = Selec_of_vector("impedance_torques")
    sel.selec(1,3)
    plug(control_torques, sel.signal('sin'))
    return sel.signal('sout')

def ele_mat_mul(mat1, mat2):
    ## elements wise multiplication
    op = Multiply_double_vector("")
    plug(mat1, op.sin1)
    plug(mat2, op.sin2)
    return op.sout

def constVector(val):
    op = VectorConstant("").sout
    op.value = list(val)
    return op

def matrixConstant(val):
    op = MatrixConstant("").sout
    op.value = val
    return op

def hom2pos(sig):
    conv_pos = MatrixHomoToPose("")
    plug(sig, conv_pos.signal('sin'))
    return conv_pos.signal('sout')

def remove_base_pos(vec1, vec2):
    op = Stack_of_vector("clean")
    op.selec1(0, 1)
    op.selec2(2, 3)
    plug(vec1, op.signal('sin1'))
    plug(vec2, op.signal('sin2'))
    return op.signal('sout')

def impedance_controller(robot_dg, gain_value,des_pos):
    ## Impdance control implementation
    xyzpos_hip = hom2pos(robot_dg.pos_hip)
    xyzpos_foot = hom2pos(robot_dg.pos_foot)
    # relative foot position to hip
    rel_pos_foot = compute_pos_diff(xyzpos_foot, xyzpos_hip, "rel_pos_foot")
    ## removing the values of the base
    ##rel_pos_foot = remove_base_pos(rel_pos_foot,rel_pos_foot)

    jac = robot_dg.jac_contact
    pos_error = compute_pos_diff(rel_pos_foot, des_pos, "pos_error")
    ## adding force in fz and also rotation forces for proper jacobian multiplication
    pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0, 0.0]), 3, 3)
    mul_double_vec_op = Multiply_double_vector("gain_multiplication")
    plug(gain_value, mul_double_vec_op.sin1)
    plug(pos_error, mul_double_vec_op.sin2)
    pos_error_with_gains = mul_double_vec_op.sout
    control_torques = compute_control_torques(jac, pos_error_with_gains)

    return control_torques


#from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("PDController")
pd.Kp.value = [0., 0.]
pd.Kd.value = [0.1,0.1]
pd.desiredposition.value = (0., 0.)
pd.desiredvelocity.value = (0., 0.)
robot_dg.acceleration.value = 3 * (0.0, )

plug(stack_zero(robot.device.signal('joint_positions')), robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities')), robot_dg.velocity)


# For making gain input dynamic through terminal
add = Add_of_double('mult')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout
control_torques = impedance_controller(robot_dg, gain_value, constVector([0.0, 0.0, -0.25]))
#if control_torques[0] < 0:
plug(control_torques, robot.device.ctrl_joint_torques)

############ Plotting #############################################

robot.add_trace("rel_pos_foot", "sout")
robot.add_ros_and_trace("rel_pos_foot", "sout")

robot.add_trace("pos_error", "sout")
robot.add_ros_and_trace("pos_error", "sout")
