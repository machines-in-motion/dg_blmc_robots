import time

from dynamic_graph import plug
from dynamic_graph.sot.core import*
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


from  dynamic_graph.sot.tools import CubicInterpolation

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


def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')

filename_pos = os.path.abspath('teststand_squatting_positions.dat')
filename_vel = os.path.abspath('teststand_squatting_velocities.dat')
file_exists(filename_pos)
file_exists(filename_vel)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '110'
reader_vel.selec.value = '110'

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

pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = (5., 5.)
pd.Kd.value = (0.1, 0.1,)

pd.desiredposition.value = (0., 0.,)
pd.desiredvelocity.value = (0., 0.,)

plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

# Expose the entity's signal to ros and the tracer together.
robot.add_ros_and_trace("PDController", "desiredposition")
robot.add_ros_and_trace("PDController", "desiredvelocity")

def start():
    reader_pos.rewind()
    reader_vel.rewind()
    reader_pos.vector.recompute(0)
    interp = CubicInterpolation('')
    interp.init.value = robot.device.joint_positions.value
    interp.goal.value = reader_pos.vector.value
    interp.sout.recompute(robot.device.joint_positions.time)
    interp.setSamplingPeriod(0.001)
    interp.start(1.0)
    plug(interp.sout, pd.desiredposition)
    plug(interp.soutdot, pd.desiredvelocity)
    time.sleep(2.)
    plug(reader_pos.vector, pd.desiredposition)
    plug(reader_vel.vector, pd.desiredvelocity)


