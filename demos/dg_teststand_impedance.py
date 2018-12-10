import numpy as np
np.set_printoptions(suppress=True, precision=2)

import pinocchio as se3
from py_dynamics_simulator.robot_wrapper import RobotWrapper

import dynamic_graph as dg
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
import dynamic_graph.sot.core as score
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


robot_py = RobotWrapper({'urdf':'hopper_1d.urdf'})
robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'joint_hip')
robot_dg.createPosition('pos_contact', 'joint_contact')

def op2(op_clazz, sin1, sin2, entity_name=""):
    op = op_clazz(entity_name)
    dg.plug(sin1, op.sin1)
    dg.plug(sin2, op.sin2)
    return op.sout

def op1(op_clazz, sin1, entity_name=""):
    op = op_clazz(entity_name)
    dg.plug(sin1, op.sin)
    return op.sout

def constVector(val):
    op = score.VectorConstant("").sout
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

print("test")

# Plug the position and velocity from the robot into the robot_dg.
dg.plug(stack_zero(robot.device.signal('joint_positions')), robot_dg.position)
dg.plug(stack_zero(robot.device.signal('joint_velocities')), robot_dg.velocity)
robot_dg.acceleration.value = 3 * (0.0, )

sjac_contact = robot_dg.jac_contact
spos_hip = hom2pos(robot_dg.pos_hip)
spos_contact = hom2pos(robot_dg.pos_contact)

# Compute the desired position. Get the value of the first slider and add
# a desired position offset for the overall desired position.
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 20
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0/float(filter_size))
# we plug the centered sliders output to the input of the filter.
dg.plug(robot.device.slider_positions, slider_filtered.sin)


slider_1_op = score.Component_of_vector("")
slider_1_op.setIndex(0)
dg.plug(slider_filtered.sout, slider_1_op.sin)
sslider_1 = slider_1_op.sout

sslider_1_scale = constVector([0., 0., -0.2,])
spos_offset = constVector([0.02, 0., 0.32])
sslider_1_scaled = op2(score.Multiply_double_vector, sslider_1, sslider_1_scale)

spos_des = op2(score.Add_of_vector, spos_offset, sslider_1_scaled, "pos_des")


# Computes the actual length difference and the difference to the desired length.
# Converts the result into a 6d wrench vector.
spos_diff = stack_to_wrench(pos_diff(spos_des, pos_diff(spos_hip, spos_contact)))

# Comptues the torque using J.T lam

storque = impedance_torque(sjac_contact, spos_diff)


# HACK: Use a PD controller to fuse the P and D signal. Adding them
# by jviereck didn't work :/ -> control process crashes.

from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("")
pd.displaySignals()
pd.Kd.value = (0.1, 0.3,)
pd.Kp.value = (150., 150.)
pd.desiredposition.value = (0., 0.)
pd.desiredvelocity.value = (0., 0.)
dg.plug(storque, pd.position)
dg.plug(robot.device.joint_velocities, pd.velocity)

dg.plug(pd.control, robot.device.ctrl_joint_torques)


# robot.add_trace('pos_des', 'sout')
# robot.add_trace('impedance_torque', 'sout')

if False:
    # # Scale the torque
    p_gain_op = score.Multiply_double_vector("")
    p_gain = p_gain_op.sin1
    p_gain.value = -50.
    dg.plug(storque, p_gain_op.sin2)

    d_gain_op = score.Multiply_double_vector("")
    d_gain = d_gain_op.sin1
    d_gain.value = -1.
    dg.plug(robot.device.joint_velocities, d_gain_op.sin2)

    # add_op = score.Add_of_vector("")
    # dg.plug(p_gain_op.sout, add_op.sin1)
    # dg.plug(d_gain_op.sout, add_op.sin2)
    # dg.plug(add_op.sout, robot.device.ctrl_joint_torques)

    stau = op2(score.Add_of_vector, d_gain_op.sout, p_gain_op.sout)
    # stau = op2(score.Substract_of_vector, p_gain_op.sout, p_gain_op.sout)
    # # stau = p_gain_op.sout


    # print(stau.value)
    # print(robot.device.ctrl_joint_torques.value)

    # stau.recompute(0)
    # print(stau.value)

    dg.plug(stau, robot.device.ctrl_joint_torques)
# dg.plug(p_gain_op.sout, robot.device.ctrl_joint_torques)
# dg.plug(d_gain_op.sout, robot.device.ctrl_joint_torques)

print("test")
