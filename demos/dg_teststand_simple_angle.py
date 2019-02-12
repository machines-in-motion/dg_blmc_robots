import numpy as np
np.set_printoptions(suppress=True, precision=2)

import pinocchio as se3
from py_dynamics_simulator.robot_wrapper import RobotWrapper

from dynamic_graph import plug
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double

from py_robot_properties_teststand.config import TeststandConfig

robot_py = TeststandConfig.buildRobotWrapper()
robot_dg = dp.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'HFE')
robot_dg.createPosition('pos_contact', 'END')

def op2(op_clazz, sin1, sin2, entity_name=""):
    op = op_clazz(entity_name)
    plug(sin1, op.sin1)
    plug(sin2, op.sin2)
    return op.sout

def op1(op_clazz, sin1, entity_name=""):
    op = op_clazz(entity_name)
    plug(sin1, op.sin)
    return op.sout

def constVector(val):
    op = VectorConstant("").sout
    op.value = list(val)
    return op

def stack_zero(sig):
    zero = VectorConstant("")
    zero.sout.value = (0.,)

    op = Stack_of_vector("")
    op.selec1(0, 1)
    op.selec2(0, 2)
    plug(zero.sout, op.sin1)
    plug(sig, op.sin2)
    return op.sout

def stack_to_wrench(sig):
    zero3 = VectorConstant("")
    zero3.sout.value = (0., 0., 0.,)

    op = Stack_of_vector("")
    op.selec1(0, 3)
    op.selec2(0, 3)
    plug(sig, op.signal('sin1'))
    plug(zero3.signal('sout'), op.signal('sin2'))
    return op.signal('sout')

def hom2pos(sig):
    conv_pos = MatrixHomoToPose("")
    plug(sig, conv_pos.signal('sin'))
    return conv_pos.signal('sout')

def pos_diff(sig1, sig2):
    sub_op = Substract_of_vector("")
    plug(sig1, sub_op.signal('sin1'))
    plug(sig2, sub_op.signal('sin2'))
    return sub_op.sout

def impedance_torque(sjac, swrench):
    op = MatrixTranspose("")
    plug(sjac, op.sin)
    sjacT = op.sout

    op = Multiply_matrix_vector('mv')
    plug(sjacT, op.signal('sin1'))
    plug(swrench, op.signal('sin2'))

    # Only keep the last two entries.
    sel = Selec_of_vector('impedance_torque')
    sel.selec(1, 3)
    plug(op.signal('sout'), sel.signal('sin'))
    return sel.signal('sout')

print("test")

# Plug the position and velocity from the robot into the robot_dg.
plug(stack_zero(robot.device.signal('joint_positions')), robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities')), robot_dg.velocity)
robot_dg.acceleration.value = 3 * (0.0, )

sjac_contact = robot_dg.jac_contact
spos_hip = hom2pos(robot_dg.pos_hip)
spos_contact = hom2pos(robot_dg.pos_contact)

# Compute the desired position. Get the value of the first slider and add
# a desired position offset for the overall desired position.
slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
filter_size = 200
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0/float(filter_size))
# we plug the centered sliders output to the input of the filter.
plug(robot.device.slider_positions, slider_filtered.sin)


slider_1_op = Component_of_vector("")
slider_1_op.setIndex(0)
plug(slider_filtered.sout, slider_1_op.sin)
sslider_1 = slider_1_op.sout

max_range = 0.75 * np.pi
sslider_1_scale = constVector([max_range, -2 * max_range])
spos_offset = constVector([-0.5 * max_range, max_range])
sslider_1_scaled = op2(Multiply_double_vector, sslider_1, sslider_1_scale)

joint_des = op2(Add_of_vector, spos_offset, sslider_1_scaled, "pos_des")

from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = (1., 1.)
pd.Kd.value = (0.1, 0.1,)
plug(joint_des, pd.desiredposition)
pd.desiredvelocity.value = (0., 0.)
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

# Create a filter for the joint velocity. Estimate the joint velocity by
# differentiating the filtered position.
cmd_filter = FIRFilter_Vector_double("cmd_filter")
cmd_filter_size = 51
cmd_filter.setSize(cmd_filter_size)
for i in range(cmd_filter_size):
    cmd_filter.setElement(i, 1.0/float(cmd_filter_size))
plug(pd.control, cmd_filter.sin)

robot.device.after.addSignal(cmd_filter.name + '.sout')
robot.add_ros_and_trace(cmd_filter.name, 'sout')

def filter_cmd_mean():
    pd.Kd.value = (0.0, 0.,)
    plug(cmd_filter.sout, robot.device.ctrl_joint_torques)
    print("Using filter on the cmd signal.")
