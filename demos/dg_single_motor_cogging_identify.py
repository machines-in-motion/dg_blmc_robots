import numpy as np
np.set_printoptions(suppress=True, precision=2)

import pickle
import datetime, threading, time

import pinocchio as se3
from py_dynamics_simulator.robot_wrapper import RobotWrapper

import dynamic_graph as dg
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
import dynamic_graph.sot.core as score
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


def op2(op_clazz, sin1, sin2, entity_name=""):
    op = op_clazz(entity_name)

    # Workaround for bug: https://github.com/stack-of-tasks/sot-core/issues/70
    if hasattr(op, 'sin0'):
        dg.plug(sin1, op.sin0)
        dg.plug(sin2, op.sin1)
    else:
        dg.plug(sin1, op.sin1)
        dg.plug(sin2, op.sin2)
    return op.sout

def op1(op_clazz, sin1, entity_name=""):
    op = op_clazz(entity_name)
    dg.plug(sin1, op.sin)
    return op.sout

def constVector(val, name=""):
    op = score.VectorConstant(name).sout
    op.value = list(val)
    return op

def stack_zero(sig):
    zero = score.VectorConstant("")
    zero.sout.value = (0.,)

    op = score.Stack_of_vector("")
    op.selec1(0, 1)
    op.selec2(0, 2)
    dg.plug(zero.sout, op.sin1)
    dg.plug(sig, op.sin2)
    return op.sout

def stack_to_wrench(sig):
    zero3 = score.VectorConstant("")
    zero3.sout.value = (0., 0., 0.,)

    op = score.Stack_of_vector("")
    op.selec1(0, 3)
    op.selec2(0, 3)
    dg.plug(sig, op.signal('sin1'))
    dg.plug(zero3.signal('sout'), op.signal('sin2'))
    return op.signal('sout')

def hom2pos(sig):
    conv_pos = score.MatrixHomoToPose("")
    dg.plug(sig, conv_pos.signal('sin'))
    return conv_pos.signal('sout')

def pos_diff(sig1, sig2):
    sub_op = score.Substract_of_vector("")
    dg.plug(sig1, sub_op.signal('sin1'))
    dg.plug(sig2, sub_op.signal('sin2'))
    return sub_op.sout

def impedance_torque(sjac, swrench):
    op = score.MatrixTranspose("")
    dg.plug(sjac, op.sin)
    sjacT = op.sout

    op = score.Multiply_matrix_vector('mv')
    dg.plug(sjacT, op.signal('sin1'))
    dg.plug(swrench, op.signal('sin2'))

    # Only keep the last two entries.
    sel = score.Selec_of_vector('impedance_torque')
    sel.selec(1, 3)
    dg.plug(op.signal('sout'), sel.signal('sin'))
    return sel.signal('sout')


from dynamic_graph.sot.core.control_pd import ControlPD


des_vel = constVector([np.pi],"des_vel")
ff = constVector([0.],"ff")

pd = ControlPD("PDController")
# dg.plug(op2(score.Multiply_double_vector, osc_Kp.sout, constVector([1, 1])), pd.Kp)
# pd.Kp.value = ((0.02,))
# pd.Kd.value = ((0.002,))

pd.Kp.value = ((0.005,))
pd.Kd.value = ((0.0002,))

dg.plug(robot.device.joint_positions, pd.position)
dg.plug(robot.device.joint_velocities, pd.velocity)

ga = score.GradientAscent('ga')
ga.learningRate.value = 1e-3

dg.plug(des_vel, ga.gradient)
dg.plug(des_vel, pd.desiredvelocity)

dg.plug(ga.value, pd.desiredposition)

dg.plug(op2(score.Add_of_vector, pd.control, ff, ""), robot.device.ctrl_joint_torques)



