# Author: Julian Viereck, MPI Tuebingen, NYU Tandon
# Date: 30 January 2019
import numpy as np
np.set_printoptions(precision=3, suppress=True)

import pinocchio as se3
from pinocchio.utils import zero

import eigenpy
eigenpy.switchToNumpyArray()

import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_quadruped_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.operator import Stack_of_vector

from dynamic_graph.sot.torque_control.tsid_controller import TsidController

def stack_vector(sin0, size0, sin1, size1):
  op = Stack_of_vector('')
  op.selec1(0, size0)
  op.selec2(0, size1)

  plug(sin0, op.sin1)
  plug(sin1, op.sin2)
  return op.sout

# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.3
q[6] = 1.
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

pin_robot = robot.pinocchio_robot_wrapper()

# Create signal containing base and joint together.

# import pdb; pdb.set_trace()

base_pos, base_vel = robot.base_signals()

sig_q = stack_vector(base_pos, 7, robot.device.joint_positions, 8)
sig_dq = stack_vector(base_vel, 6, robot.device.joint_velocities, 8)

# Setup the control graph to track the desired joint positions.
pd = ControlPD("PDController")
pd.Kp.value = 8 * (5.,)
pd.Kd.value = 8 * (0.1,)

pd.desiredposition.value = np.array(q[7:]).T[0].tolist()
pd.desiredvelocity.value = 8 * (0.,)

plug(robot.device.joint_positions, pd.position)
plug(robot.device.joint_velocities, pd.velocity)

plug(pd.control, robot.device.ctrl_joint_torques)

tolist = lambda v: np.array(v).reshape(-1).tolist()

def build_invdyn_controller():
  controller = TsidController('tsid_quadruped')
  controller.loadURDF(robot.urdf_path)

  contact_frames = ['FL_contact', 'FR_contact', 'HL_contact', 'HR_contact']

  for cf in contact_frames:
      controller.addContact(cf)
      assert(controller.hasSignal(cf + '__pos'))

  # Set the task weights for the controller.
  controller.w_com.value = 1.0;
  controller.w_feet.value = 1.0;
  controller.w_forces.value = 1e-5
  # controller.w_posture.value = 1.0; # Enable if desired posture is provided.
  controller.weight_contact_forces.value = [1., 1., 1e-3]

  # Gains.
  def set_gain(name, value, dim):
      controller.__getattr__('kp_' + name).value = dim * [value]
      controller.__getattr__('kd_' + name).value = dim * [2. * np.sqrt(value)]

  set_gain('com', 1., 3)
  set_gain('feet', 1., 6)
  set_gain('constraints', 100., 6)

  # Signals for the contacts.
  for cf in contact_frames:
      controller.__getattr__(cf + '__f_min').value = 0.
      controller.__getattr__(cf + '__f_max').value = 10.
      controller.__getattr__(cf + '__contact_normal').value = [0., 0., 1]
      controller.__getattr__(cf + '__mu').value = 0.6

  # Use the current com position as desired position and no motion.
  com_initial = pin_robot.com(q)
  controller.com_ref_pos.value = tolist(com_initial)
  controller.com_ref_vel.value = 3 * [0.]
  controller.com_ref_acc.value = 3 * [0.]

  # Plug the q and dq to the controller.
  plug(sig_q, controller.q)
  plug(sig_dq, controller.v)

  # Init the controller
  dt = 1e-3
  controller.init(dt)

  return controller

# Run the robot (here the simulator) for 5000 steps (= 5 seconds). The second
# optional parameter specifies how much to sleep every 1/60. steps. This allows
# a plugged visualizer to display the evolving trajectory.
robot.run(1000, 0.)

# Setup the inverse dynamics controller and update the desired com based on the
# current position.
controller = build_invdyn_controller()
robot.device.after.addSignal('{}.tau_des'.format(controller.name))

com_initial = pin_robot.com(np.matrix(sig_q.value).T)
controller.com_ref_pos.value = tolist(com_initial)

plug(controller.tau_des, robot.device.ctrl_joint_torques)

robot.run(1000, 0.)

com_final = pin_robot.com(np.matrix(sig_q.value).T)

print('com initial:', com_initial)
print('com final:', com_final)
print('--> com drift:', np.linalg.norm(com_initial - com_final))

print('tsid:', controller.tau_des.value)
print('pd:', robot.device.ctrl_joint_torques.value)

raw_input("Press Enter to continue...")
