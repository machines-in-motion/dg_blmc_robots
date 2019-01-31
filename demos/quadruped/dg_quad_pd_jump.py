import time
from dynamic_graph import plug
import dynamic_graph.sot.core as score
from dynamic_graph.sot.core.reader import Reader
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph_manager.vicon_sdk import ViconClientEntity

from  dynamic_graph.sot.tools import CubicInterpolation

###
# Create vicon tracker.
vicon_client = ViconClientEntity("vicon_client")

print("Display the initial state of the entity")
print("Commands: ", vicon_client.commands())
print("Signals: ", vicon_client.displaySignals())
print("")

host_name_quadruped = '10.32.27.53:801'
print("connect to the host")
vicon_client.connect_to_vicon(host_name_quadruped)
print("")

robot_vicon_name = 'quadruped'
print("create the signals for the {}/{}".format(robot_vicon_name, robot_vicon_name))
vicon_client.add_object_to_track("{}/{}".format(robot_vicon_name, robot_vicon_name))
print("from now the signal {} is available:".format(robot_vicon_name))
vicon_client.displaySignals()

# Trace and expose the vicon base.
robot.add_ros_and_trace("vicon_client", robot_vicon_name)

###
# Create PD controller.
pd = ControlPD("PDController")
pd.displaySignals()
pd.Kp.value = 8 * (5.,)
pd.Kd.value = 8 * (0.1,)

# Wait for the joint positions to become available and then use it as initila
# desired joint positions.
time.sleep(0.1)

pd.desiredposition.value = robot.device.joint_positions.value
pd.desiredvelocity.value = 8 * (0.,)

plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)


def build_des_pos(hip, knee):
  arr = []
  for i in range(2):
    arr.append(hip)
    arr.append(knee)
  for i in range(2):
    arr.append(-hip)
    arr.append(-knee)
  return arr

des_joints = build_des_pos(0.9, -1.3)
mid_joints = build_des_pos(1.3, -2.0)


def run_interpolation(des_joints, duration=1):
  interp = CubicInterpolation('')
  interp.init.value = robot.device.joint_positions.value
  interp.goal.value = des_joints
  print('Start from:', interp.init.value)
  print('Go to:', interp.goal.value)
  interp.sout.recompute(robot.device.joint_positions.time)
  interp.setSamplingPeriod(0.001)
  interp.start(duration)
  plug(interp.sout, pd.desiredposition)
  plug(interp.soutdot, pd.desiredvelocity)
  time.sleep(1.)

def goto_position():
  run_interpolation(des_joints)
  run_interpolation(mid_joints)
  run_interpolation(des_joints, 1.)

def goto_jump():
  run_interpolation(des_joints)
  run_interpolation(mid_joints)
  run_interpolation(des_joints, 0.05)

def record_jumps():
  robot.start_tracer()
  time.sleep(1.)
  for i in range(5):
    goto_jump()
  time.sleep(1.)
  robot.stop_tracer()
