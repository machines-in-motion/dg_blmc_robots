import numpy as np
np.set_printoptions(suppress=True, precision=2)

from dynamic_graph import plug
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.op_point_modifier import OpPointModifier
from dynamic_graph.sot.core import *
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


def op2(op_clazz, sin1, sin2, entity_name=""):
    op = op_clazz(entity_name)

    # Workaround for bug: https://github.com/stack-of-tasks/sot-core/issues/70
    if hasattr(op, 'sin0'):
        plug(sin1, op.sin0)
        plug(sin2, op.sin1)
    else:
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

# Create oscillators.
import dynamic_graph.sot.tools
osc_hip = dynamic_graph.sot.tools.Oscillator("oscillator_hip")
osc_hip.setTimePeriod(0.001)
osc_hip.omega.value = 2.75 * np.pi
osc_hip.magnitude.value = 1.0
osc_hip.bias.value = 0.0

osc_knee = dynamic_graph.sot.tools.Oscillator("oscillator_knee")
osc_knee.setTimePeriod(0.001)
osc_knee.omega.value = 3.00 * np.pi
osc_knee.magnitude.value = 1.0
osc_knee.bias.value = 0.0

def slow_motion():
    osc_hip.omega.value = 0.2 * np.pi
    osc_knee.omega.value = 0.3 * np.pi

def fast_motion():
    osc_hip.omega.value = 2.75 * np.pi
    osc_knee.omega.value = 3.0 * np.pi

osc_Kp = dynamic_graph.sot.tools.Oscillator("oscillator_Kp")
osc_Kp.setTimePeriod(0.001)
osc_Kp.omega.value = 10.0 * np.pi
osc_Kp.magnitude.value = 2.0
osc_Kp.bias.value = 3.0

max_range = 0.4 * np.pi
sslider_1_hip_scale = constVector([max_range, 0.])
sslider_1_knee_scale = constVector([0, -2. * max_range])

sslider_1_hip_scaled = op2(Multiply_double_vector,
    op2(Multiply_of_double, sslider_1, osc_hip.sout),
    sslider_1_hip_scale)
sslider_1_knee_scaled = op2(Multiply_double_vector,
    op2(Multiply_of_double, sslider_1, osc_knee.sout),
    sslider_1_knee_scale)

joint_des = op2(Add_of_vector, sslider_1_hip_scaled, sslider_1_knee_scaled, "joint_des")

from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("PDController")
# plug(op2(Multiply_double_vector, osc_Kp.sout, constVector([1, 1])), pd.Kp)
pd.Kp.value = (0., 0.)
pd.Kd.value = (0.1, 0.1,)
plug(joint_des, pd.desiredposition)
pd.desiredvelocity.value = (0., 0.)
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

def log_entity(entityName, signalNames):
    [robot.add_ros_and_trace(entityName, sigName) for sigName in signalNames]

log_entity('PDController', ['Kp', 'Kd', 'desiredvelocity', 'desiredposition'])
