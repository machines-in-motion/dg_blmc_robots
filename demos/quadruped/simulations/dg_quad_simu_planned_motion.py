## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot
from dynamic_graph.sot.core.reader import Reader

import time
import py_dg_blmc_robots
from py_dg_blmc_robots.quadruped import get_quadruped_robot


###### debug #############################################
#
# local_device = Device("hopper_robot")
# #yaml_path = os.path.join(rospkg.RosPack().get_path("robot_properties_teststand"),'/config/teststand.yaml')
# yaml_path = '/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_teststand/config/teststand.yaml'
# #print(yaml_path)
# local_device.initialize(yaml_path)
# robot = Robot(name=local_device.name, device=local_device)

###### robot init #######################################################
# Get the robot corresponding to the quadruped.

robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.4
q[6] = 1.
for i in range(4):
    q[7 + 2 * i] = 0.8
    q[8 + 2 * i] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

###############################################################################

### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')

# filename_pos = os.path.abspath('../trajectories/quadruped_squatting_positions_eff.dat')
filename_vel = os.path.abspath('../trajectories/quadruped_squatting_velocities_eff.dat')

filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"

file_exists(filename_pos)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '1111111111110'
reader_vel.selec.value = '1111111111110'

pos_des_hl = selec_vector(reader_pos.vector, 0, 3, "HL")
pos_des_hr = selec_vector(reader_pos.vector, 3, 6, "HR")
pos_des_fl = selec_vector(reader_pos.vector, 6, 9, "FL")
pos_des_fr = selec_vector(reader_pos.vector, 9,12, "FR")



###############################################################################


pos_des = constVector([0.0, 0.0, -0.25], "pos_des")
##For making gain input dynamic through terminal
add = Add_of_double('gain')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 100.0
gain_value = add.sout


#control_torques = quad_impedance_controller(robot, pos_des, pos_des, pos_des, pos_des, gain_value)

control_torques = quad_impedance_controller(robot, pos_des_fl, pos_des_fr, pos_des_hl, pos_des_hr, gain_value)

plug(control_torques, robot.device.ctrl_joint_torques)

robot.run(10000, 1./60.)


##############################################################################

# from pinocchio.utils import zero
#
# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)
#
# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)
#
# q[:] = np.asmatrix(robot.device.joint_positions.value).T
# dq[:] = np.asmatrix(robot.device.joint_velocities.value).T
#
# print(q, dq)
#
# dt = 0.001#config.dt
# motor_inertia = 0.045
#
# for i in range(4):
#     # fill the sensors
#     robot.device.joint_positions.value = q.T.tolist()[0][:]
#     robot.device.joint_velocities.value = dq.T.tolist()[0][:]
#
#     # "Execute the dynamic graph"
#     robot.device.execute_graph()
#
#
#     joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
#     #joint_torques[:] = control_torques.value
#
#     #print(q, dq)
#     pos_des_hl.recompute(i)
#     pos_des_hr.recompute(i)
#     pos_des_fl.recompute(i)
#     pos_des_fr.recompute(i)
#     print(pos_des_hl.value, pos_des_hr.value, pos_des_fl.value, pos_des_fr.value)
#
#     # integrate the configuration from the computed torques
#     #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
#     #dq = (dq + dt * joint_torques / motor_inertia)
#     q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
#     dq = (dq + dt * 0.01 / motor_inertia)
#
#
#     if (i % 1000) == 0:
#         # print "qref =     ", robot.pid_control.pose_controller.qRef.value
#         # print ("q =        ",
#         #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
#         # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
#         # print "currents = ", robot.device.ctrl_joint_torques.value
#         pass
#
#
# print
# print "End of simulation"
# print "control_torques = ", robot.device.ctrl_joint_torques.value
