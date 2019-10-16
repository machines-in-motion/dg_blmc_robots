# receding reactive LQR implementation for solo. The desired trajectory is obtained
# from the centroidal momentum controller.
#
# Author : Avadesh Meduri
# Date: 30/04/2019

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values
from dynamic_graph_manager.vicon_sdk import ViconClientEntity

from dynamic_graph_manager.dg_tools import ReactiveLQRController


###### robot init #######################################################
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

local_device = Device("hopper_robot")
yaml_path = "./../../../../robots/robot_properties/robot_properties_quadruped/config/quadruped.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)



################################################################################


w1 = 10.0
w2 = 100000.0
py_q = np.zeros((13,13))
np.fill_diagonal(py_q, w1)
# py_q[0][0] = 100000.0
# py_q[1][1] = 100000.0
# py_q[7][7] = 100000.0
# py_q[8][8] = 100000.0
# py_q[9][9] = 100000.0
# py_q[10][10] = 100000.0
# py_q[11][11] = 100000.0



py_r = np.zeros((12,12))
np.fill_diagonal(py_r, w2)

q = constMatrix(py_q, "Q")
r = constMatrix(py_r, "R")


horizon = 4
horizon_arr = constVector([horizon,],"lqr_horizon")
mass = constVector([2.2,], 'mass')
inertia = constMatrix([[0.00578574, 0.0, 0.0],
                       [0.0, 0.01938108, 0.0],
                       [0.0, 0.0, 0.02476124]], 'inertia')

################################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

################################################################################
from dynamic_graph.sot.core.reader import Reader

reader_pos_com = Reader('PositionComReader')
reader_vel_com = Reader('VelocityReader')
reader_ori_com = Reader('OrientationComReader')
reader_ang_vel_com = Reader('AngVelReader')
reader_fff = Reader('FeedForwardForceComReader')
reader_fff_1 = Reader('FeedForwardForceComReader1')
reader_end_eff = Reader('EndEffReader')
reader_end_eff_1 = Reader('EndEffReader1')
reader_cnt_value = Reader('CntReaderValue')

filename_pos_com = "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_com_with_horizon.dat"
filename_vel_com = "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_lmom_with_horizon.dat" ## this is velocity name must be changed
filename_ori_com = "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_quaternion_with_horizon.dat"
filename_ang_vel_com = "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_base_ang_velocities_with_horizon.dat"
filename_fff =  "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_forces_with_horizon_part1.dat"
filename_end_eff =  "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_abs_with_horizon_part1.dat"
filename_fff_1 =  "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_forces_with_horizon_part2.dat"
filename_end_eff_1 =  "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_abs_with_horizon_part2.dat"
filename_cnt_value =  "/home/ameduri/devel/workspace/src/catkin/control/kino-dynamic-opt/momentumopt/demos/des_contact_activation_with_horizon.dat"




file_exists(filename_pos_com)
file_exists(filename_vel_com)
file_exists(filename_ori_com)
file_exists(filename_ang_vel_com)
file_exists(filename_fff)
file_exists(filename_end_eff)
file_exists(filename_fff_1)
file_exists(filename_end_eff_1)
file_exists(filename_cnt_value)

reader_pos_com.load(filename_pos_com)
reader_vel_com.load(filename_vel_com)
reader_ori_com.load(filename_ori_com)
reader_ang_vel_com.load(filename_ang_vel_com)
reader_fff.load(filename_fff)
reader_end_eff.load(filename_end_eff)
reader_fff_1.load(filename_fff_1)
reader_end_eff_1.load(filename_end_eff_1)
reader_cnt_value.load(filename_cnt_value)

reader_pos_com.selec.value = (3*(horizon + 1))*'1' + '0'
reader_vel_com.selec.value = (3*(horizon + 1))*'1' + '0'
reader_ori_com.selec.value = (4*(horizon + 1))*'1' + '0'
reader_ang_vel_com.selec.value = (3*(horizon + 1))*'1' + '0'
reader_fff.selec.value = (12*(2 + 1))*'1' + '0'
reader_end_eff.selec.value = (12*(2 + 1))*'1' + '0'
reader_fff_1.selec.value = (12*(1 + 1))*'1'
reader_end_eff_1.selec.value = (12*(1 + 1))*'1'
reader_cnt_value.selec.value = (4*(horizon + 1))*'1' + '0'

des_fff = stack_two_vectors(reader_fff.vector, reader_fff_1.vector, 36, 24)
des_end_eff = stack_two_vectors(reader_end_eff.vector, reader_end_eff_1.vector, 36, 24)

################################################################################
#
reactive_lqr_ctrl = ReactiveLQRController("lqr_ctrl")
plug(reader_pos_com.vector, reactive_lqr_ctrl.com_pos)
plug(reader_vel_com.vector, reactive_lqr_ctrl.com_vel)
plug(reader_ori_com.vector, reactive_lqr_ctrl.com_ori)
plug(reader_ang_vel_com.vector, reactive_lqr_ctrl.com_ang_vel)
plug(des_end_eff, reactive_lqr_ctrl.end_eff_pos)
plug(des_fff, reactive_lqr_ctrl.des_fff)
plug(reader_cnt_value.vector, reactive_lqr_ctrl.cnt_value)

plug(q, reactive_lqr_ctrl.q)
plug(r, reactive_lqr_ctrl.r)
plug(horizon_arr, reactive_lqr_ctrl.horizon)
plug(mass, reactive_lqr_ctrl.mass)
plug(inertia, reactive_lqr_ctrl.inertia)

lqr_gains = reactive_lqr_ctrl.return_lqr_gains

################################################################################


######## robot simulation ################################################

from pinocchio.utils import zero

q = zero(8)
dq = zero(8)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(8)
joint_torques = zero(8)

q = zero(8)
dq = zero(8)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(8)
joint_torques = zero(8)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

dt = 0.001#config.dt
motor_inertia = 0.045

for i in range(500,600):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()

    # reader_ori_com.vector.recompute(i)
    # print(np.shape(reader_ori_com.vector.value))

    lqr_gains.recompute(i)
    # print(lqr_gains.value)
    print('\n')


    # integrate the configuration from the computed torques
    #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
    #dq = (dq + dt * joint_torques / motor_inertia)
    q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
    dq = (dq + dt * 0.01 / motor_inertia)


    if (i % 1000) == 0:
        pass
