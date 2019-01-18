import numpy as np
np.set_printoptions(suppress=True, precision=2)

from dynamic_graph import plug
from dynamic_graph.sot.core import *

from dynamic_graph.sot.core.control_pd import ControlPD

pd = ControlPD("PDController")

pd.Kp.value = ((0.005,))
pd.Kd.value = ((0.0002,))
pd.desiredposition = ((0.0,)) # you can adjust this directly in the REPL
pd.desiredvelocity = ((0.0,))

plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)