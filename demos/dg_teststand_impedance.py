import numpy as np
np.set_printoptions(suppress=True, precision=2)

from scipy.signal import savgol_coeffs

import pinocchio as se3
from py_dynamics_simulator.robot_wrapper import RobotWrapper

# import dynamic_graph as dg
from dynamic_graph import plug
import dynamic_graph.sot.dynamics_pinocchio as dp
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
# import dynamic_graph.sot.core as score
from dynamic_graph.sot.core import *
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
filter_size = 20
slider_filtered.setSize(filter_size)
for i in range(filter_size):
    slider_filtered.setElement(i, 1.0/float(filter_size))
# we plug the centered sliders output to the input of the filter.
plug(robot.device.slider_positions, slider_filtered.sin)


slider_1_op = Component_of_vector("")
slider_1_op.setIndex(0)
plug(slider_filtered.sout, slider_1_op.sin)
sslider_1 = slider_1_op.sout

sslider_1_scale = constVector([0., 0., -0.2,])
spos_offset = constVector([0.02, 0., 0.32])
sslider_1_scaled = op2(Multiply_double_vector, sslider_1, sslider_1_scale)

# Adding an oscillator to the game.
import dynamic_graph.sot.tools
osc = dynamic_graph.sot.tools.Oscillator("oscillator")
osc.setTimePeriod(0.001)
osc.omega.value = 0.5 * np.pi

def start_oscillator():
    osc.magnitude.value = 0.25
    osc.bias.value = 0.75

def stop_oscillator():
    osc.magnitude.value = 0.0
    osc.bias.value = 1.0

def oscilate_frequency():
    osc_Kp = dynamic_graph.sot.tools.Oscillator("oscillator_Kp")
    osc_Kp.setTimePeriod(0.001)
    osc_Kp.omega.value = 0.1
    osc_Kp.magnitude.value = 0.2
    osc_Kp.bias.value = 0.5

    plug(osc_Kp.sout, osc.omega)

stop_oscillator()

spos_des = op2(Add_of_vector, spos_offset, sslider_1_scaled, "pos_des")

spos_des_osc_op = Multiply_double_vector("dist_osc")
plug(osc.sout, spos_des_osc_op.sin1)
plug(spos_des, spos_des_osc_op.sin2)


# Computes the actual length difference and the difference to the desired length.
# Converts the result into a 6d wrench vector.
spos_diff = stack_to_wrench(pos_diff(spos_des_osc_op.sout, pos_diff(spos_hip, spos_contact)))

# Add a gain for the position error to convert into a desired force.
force_des_op = Multiply_double_vector("force_des")
f_gain = force_des_op.sin1
f_gain.value = -10.
plug(spos_diff, force_des_op.sin2)

# Comptues the torque using J.T lam

storque = impedance_torque(sjac_contact, force_des_op.sout) # TODO: rename to ctrl_torque?


from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("PDController")
pd.Kp.value = (0., 0.) # since we are now applying PD to the position, set gain to 0
pd.Kd.value = (0.1, 0.1,)

pd.desiredposition.value = (0., 0.)
pd.desiredvelocity.value = (0., 0.)
# plug(storque, pd.position)
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

# plug(pd.control, robot.device.ctrl_joint_torques)
plug(op2(Add_of_vector, pd.control, storque), robot.device.ctrl_joint_torques)

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
    f_gain.value = 1.
    plug(cmd_filter.sout, robot.device.ctrl_joint_torques)
    print("Using filter on the cmd signal.")


# robot.add_ros_and_trace('oscillator', 'sout')
robot.add_trace('oscillator', 'sout')
robot.add_ros_and_trace('force_des', 'sout')
robot.add_ros_and_trace('impedance_torque', 'sout')

def log_entity(entityName, signalNames):
    [robot.add_ros_and_trace(entityName, sigName) for sigName in signalNames]

log_entity('PDController', ['Kp', 'Kd', 'position', 'velocity',
                            'desiredvelocity', 'desiredposition', 'control'])


print("test")
