import dynamic_graph as dg
import dynamic_graph.sot.core as score
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')

filename_pos = os.path.abspath('devel_dg/workspace/src/catkin/robots/dg_blmc_robots/demos/teststand_squatting_positions.dat')
filename_vel = os.path.abspath('devel_dg/workspace/src/catkin/robots/dg_blmc_robots/demos/teststand_squatting_velocities.dat')
file_exists(filename_pos)
file_exists(filename_vel)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '110'
reader_vel.selec.value = '110'

pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = (5., 5.)
pd.Kd.value = (0.1, 0.1,)

dg.plug(reader_pos.vector, pd.desiredposition)
dg.plug(reader_vel.vector, pd.desiredvelocity)

dg.plug(robot.device.joint_positions, pd.position)
dg.plug(robot.device.joint_velocities, pd.velocity)

dg.plug(pd.control, robot.device.ctrl_joint_torques)

# Expose the entity's signal to ros and the tracer together.
robot.add_ros_and_trace("PDController", "desiredposition")
robot.add_ros_and_trace("PDController", "desiredvelocity")
