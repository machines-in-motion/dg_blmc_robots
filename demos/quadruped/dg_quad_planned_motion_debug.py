## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader



from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot


###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_teststand/config/teststand.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)




def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

#############################################################################
### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_com = Reader('ComReader')
reader_lmom = Reader('Lmom')
reader_amom = Reader('Amom')
reader_forces = Reader('forces')
reader_lqr1 = Reader('lqr1')
reader_lqr2 = Reader('lqr2')
reader_lqr3 = Reader('lqr3')



filename_pos = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_velocities_eff.dat"
filename_com = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_com.dat"
filename_lmom = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_lmom.dat"
filename_amom = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_amom.dat"
filename_forces = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_forces.dat"
filename_lqr1 = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr1.dat"
filename_lqr2 = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr2.dat"
filename_lqr3 = "/home/ameduri/devel_blmc/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr3.dat"

tmp = np.loadtxt(filename_lqr1)


file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_com)
file_exists(filename_lmom)
file_exists(filename_amom)
file_exists(filename_forces)
file_exists(filename_lqr1)
file_exists(filename_lqr2)
file_exists(filename_lqr3)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_com.load(filename_com)
reader_lmom.load(filename_lmom)
reader_amom.load(filename_amom)
reader_forces.load(filename_forces)
reader_lqr1.load(filename_lqr1)
reader_lqr2.load(filename_lqr2)
reader_lqr3.load(filename_lqr3)


# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_com.selec.value = '1110'
reader_lmom.selec.value = '1110'
reader_amom.selec.value = '1110'
reader_forces.selec.value = '1111111111110'
reader_lqr1.selec.value = 36*'1'
reader_lqr2.selec.value = 36*'1'
reader_lqr3.selec.value = 36*'1'

len(reader_lqr1.selec.value)

des_pos = reader_pos.vector
des_vel = reader_vel.vector
des_com = reader_com.vector
des_lmom = reader_com.vector
des_amom = reader_amom.vector
des_forces = reader_forces.vector
### sot_reader can read a vector of max size 40.
des_lqr1 = reader_lqr1.vector
des_lqr2 = reader_lqr2.vector
des_lqr3 = reader_lqr3.vector

des_lqr = stack_two_vectors(des_lqr1, des_lqr2, 36, 36)
des_lqr = stack_two_vectors(des_lqr, des_lqr3, 72, 36)

###############################################################################

quad_com_ctrl = quad_com_control(robot)
f_lqr = quad_com_ctrl.return_lqr_tau(des_com, des_lmom, des_amom, des_forces, des_lqr)
quad_com_ctrl.record_data()

######## robot simulation ################################################

from pinocchio.utils import zero

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

print(q, dq)

dt = 0.001#config.dt
motor_inertia = 0.045

tmp = []

for i in range(0, 3000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()


    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
    #joint_torques[:] = control_torques.value

    # quad_com_ctrl.delta_f.recompute(i)
    # print(quad_com_ctrl.delta_f.value)
    #print(q, dq)

    # f_lqr.recompute(i)
    # print(f_lqr.value)

    # quad_com_ctrl.f_thr.recompute(i)
    # print(quad_com_ctrl.f_thr.value)

    des_lqr.recompute(i)
    # print(np.shape(des_lqr.value))
    # print(des_lqr.value[0:9])
    # assert False
    tmp.append(des_lqr.value)

    # integrate the configuration from the computed torques
    #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
    #dq = (dq + dt * joint_torques / motor_inertia)
    q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
    dq = (dq + dt * 0.01 / motor_inertia)


    if (i % 1000) == 0:
        # print "qref =     ", robot.pid_control.pose_controller.qRef.value
        # print ("q =        ",
        #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
        # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
        # print "currents = ", robot.device.ctrl_joint_torques.value
        pass


from matplotlib import pyplot as plt
lqr = np.array(tmp)

n = 9*8
fig1, ax1 = plt.subplots(2,1)
ax1[0].plot(lqr[:, 0+n], label = "lqr_x_traj")
ax1[0].plot(lqr[:, 1+n], label = "lqr_y_traj")
ax1[0].plot(lqr[:, 2+n], label = "lqr_z_traj")
ax1[0].set_xlabel("millisec")
ax1[0].set_ylabel("m")
ax1[0].legend()
ax1[0].grid()


ax1[1].plot(lqr[:, 3+n], label = "lqr_xd_traj")
ax1[1].plot(lqr[:, 4+n], label = "lqr_yd_traj")
ax1[1].plot(lqr[:, 5+n], label = "lqr_zd_traj")
ax1[1].set_xlabel("millisec")
ax1[1].set_ylabel("m")
ax1[1].legend()
ax1[1].grid()

plt.show()
