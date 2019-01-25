## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 23 January 2019

import os, os.path

import numpy as np
import rospkg

import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper

from dynamic_graph import plug
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


urdf_path = os.path.join(rospkg.RosPack().get_path("dg_blmc_robots"),'demo/hopper_1d.urdf')
robot_py = se3.RobotWrapper(urdf_path,[os.path.join(rospkg.RosPack().get_path("robot_properties_quadruped"), 'urdf')])
#robot_py = se3.RobotWrapper(urdf_path,[os.path.join(rospkg.RosPack().get_path("robot_properties_quadruped"), 'urdf')])
robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('Jac', 'contact')
robot_dg.createPosition('pos_hip', 'joint_hip')
robot_dg.createPosition('pos_foot', 'joint_contact')

def compute_pos_error(pos, pos_des):
    sub_op = Substract_of_vector("")
    plug(pos, sub_op.signal('sin1'))
    plug(pos_des, sub_op.signal('sin2'))
    return sub_op.sout

def Mat_transpose(mat):
    op = MatrixTranspose("")
    plug(mat, op.sin)
    return op.sout

def Mat_multiply(mat1,mat2):
    mat_mul = Multiply_matrix_vector("mv")
    plug(mat1, mat_mul.signal('sin1'))
    plug(mat2, mat_mul.signal('sin2'))
    return mat_mul.sout

def compute_control_torques(jac,errors, gains):
    ##Transpose [tau1, tau2] = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T

    jacT = Mat_transpose(jac)
    errors_with_gains = Mat_multiply(gains, errors)
    control_torques = Mat_multiply(jacT,errors_wit_gains)

    ## selecting only 2nd and 3rd element in torques as element one represent base acceleration
    sel = Selec_of_vector("impedance_torques")
    sel.selec(1,3)
    plug(control_torques, sel.signal('sin'))
    return sel.signal('sout')

#from dynamic_graph.sot.core.control_pd import ControlPD

#pd = ControlPD("PDController")
#pd.Kp.value = (0., 0.)
#pd.Kd.value = (0.1,0.1)

print(robot.device.signal('joint_positions'))

#plug(robot.device.signal('joint_positions'), robot_dg.position)
#plug(robot.device.signal('joint_velocities'), robot_dg.velocity)
#robot_dg.acceleration.value = 3 * (0.0, )
