import time
from dynamic_graph import plug
import dynamic_graph.sot.core as score
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD

from  dynamic_graph.sot.tools import CubicInterpolation

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

def flip_sign(sig):
    op = score.operator.Multiply_of_vector('')

    op.sin0.value = [1, 1, 1, 1, -1, -1, -1, -1]
    plug(sig, op.sin1)
    return op.sout

reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')

filename_pos = os.path.abspath('data/quadruped_squatting_positions.dat')
filename_vel = os.path.abspath('data/quadruped_squatting_velocities.dat')
file_exists(filename_pos)
file_exists(filename_vel)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111110'
reader_vel.selec.value = '111111110'

pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = 8 * (5.,)
pd.Kd.value = 8 * (0.1,)

time.sleep(0.5)

pd.desiredposition.value = robot.device.joint_positions.value
pd.desiredvelocity.value = 8 * (0.,)

# plug(reader_pos.vector, pd.desiredposition)
# plug(reader_vel.vector, pd.desiredvelocity)

# Flip the sign for the reader.
reader_pos_vector = flip_sign(reader_pos.vector)
reader_vel_vector = flip_sign(reader_vel.vector)

plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

# Expose the entity's signal to ros and the tracer together.
robot.add_ros_and_trace("PDController", "desiredposition")
robot.add_ros_and_trace("PDController", "desiredvelocity")

def start():
    reader_pos.rewind()
    reader_vel.rewind()
    reader_pos_vector.recompute(0)
    interp = CubicInterpolation('')
    interp.init.value = robot.device.joint_positions.value
    interp.goal.value = reader_pos_vector.value
    interp.sout.recompute(robot.device.joint_positions.time)
    interp.setSamplingPeriod(0.001)
    interp.start(1.0)
    plug(interp.sout, pd.desiredposition)
    plug(interp.soutdot, pd.desiredvelocity)
    time.sleep(2.)
    plug(reader_pos_vector, pd.desiredposition)
    plug(reader_vel_vector, pd.desiredvelocity)





