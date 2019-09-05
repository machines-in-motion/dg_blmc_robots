## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 12/02/19



import pinocchio as se3
from pinocchio.utils import zero

from leg_impedance_control.utils import constVector
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import *

############################################################################

#from leg_impedance_control.utils import *
#from leg_impedance_control.traj_generators import *

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_dg_blmc_robots.teststand import TeststandConfig


# Get the robot corresponding to the quadruped.
robot = get_teststand_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.8
q[2] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

###################################################################################

entityName = "hopper"
osc_pos = dynamic_graph.sot.tools.Oscillator(entityName + "_pos")
osc_pos.setTimePeriod(0.001)
osc_pos.omega.value = 1.0*np.pi
osc_pos.magnitude.value = 0.06
osc_pos.phase.value = 0.0
osc_pos.bias.value = -0.22

osc_vel = dynamic_graph.sot.tools.Oscillator(entityName + '_vel')
osc_vel.setTimePeriod(0.001)
osc_vel.omega.value = osc_pos.omega.value
osc_vel.magnitude.value = osc_pos.magnitude.value*osc_pos.omega.value
osc_vel.phase.value = osc_pos.phase.value + np.pi/2.0
osc_vel.bias.value = 0

unit_vector_pos = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_pos")
unit_vector_vel = constVector([0.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vector_vel")

des_pos = mul_double_vec_2(osc_pos.sout, unit_vector_pos, "des_position")
des_vel = mul_double_vec_2(osc_vel.sout, unit_vector_vel, "des_velocity")




########################################################################################################

kp = constVector([100.0, 0.0, 100.0, 0.0, 0.0, 0.0], "kp")
kd = constVector([1.0  , 0.0, 1.0  , 0.0, 0.0, 0.0], "kd")

#######################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)
#########################################################################

robot.run(10000, 1./60.)
