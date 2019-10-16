from dynamic_graph import plug
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD

# reader = Reader('')
reader = Reader('DataReader')

filename = os.path.abspath('test_data_point.dat')
if os.path.isfile(filename):
    print("The file %s exists" %filename)
else:
    print("The file %s does not exist" %filename)

print("Reading data File:")
# reader.load(file)
reader.load(filename)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader.selec.value = '1100'

# for i in range(5):
#     reader.vector.recompute(i)
#     print(reader.vector.time)
#     print(reader.vector.value)

# reader.vector.recompute(0)

pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = (5., 5.)
pd.Kd.value = (0.1, 0.1,)

# plug(joint_des, pd.desiredposition)
plug(reader.vector, pd.desiredposition)

pd.desiredvelocity.value = (0., 0.)
plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

# Expose the entity's signal to ros and the tracer together.
robot.add_ros_and_trace("PDController", "desiredposition")



