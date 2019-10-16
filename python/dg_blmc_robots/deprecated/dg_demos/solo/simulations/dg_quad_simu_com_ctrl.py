#impedance controller implementation for COM (used for quadruped)
#Author : Avadesh Meduri
#Date : 25/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import *

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "./../../../../../robots/robot_properties/robot_properties_teststand/config/teststand.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)

################################################################################

kp_com = constVector([50.0, 0.0, 50.0], "kp_com")
kd_com = constVector([0.5, 0.0, 0.5], "kd_com")
kp_ang_com = constVector([1.0, 1.0, 1.0], "kp_ang_com")
des_pos_com = constVector([0.0, 0.0, 0.0], "des_pos_com")
des_vel_com = constVector([0.0, 0.0, 0.0], "des_vel_com")
des_fff_com = constVector([0.0, 0.0, 2.2*9.81], "des_fff_com")
des_omega_com = constVector([0.0, 0.0, 0.0], "des_com_omega")
des_fft_com = constVector([0.0, 0.0, 0.0], 'des_fft_com')

pos_des = constVector([0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0,
                       0.0, 0.0, -0.22, 0.0, 0.0, 0.0],
                        "des_pos")

vel_des = zero_vec(24, "des_vel")


################################################################################
quad_com_ctrl = quad_com_control(robot)
com_tau = quad_com_ctrl.compute_torques(kp_com, des_pos_com, kd_com, des_vel_com,
                                                                    des_fff_com)

tau_per_leg = quad_com_ctrl.return_com_torques(com_tau)

ang_tau = quad_com_ctrl.compute_ang_control_torques(kp_ang_com, des_omega_com, des_fft_com)

#################################################################################
##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

# quad_imp_ctrl = quad_leg_impedance_controller(robot)
# control_torques = quad_imp_ctrl.return_control_torques(kp_split, pos_des,
#                                                 kd_split, vel_des, kf, tau_per_leg)
# # plug(control_torques, robot.device.ctrl_joint_torques)



################################################################################
quad_com_ctrl.record_data()
###############################################################################

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

dt = 0.001#config.dt
motor_inertia = 0.045

for i in range(40000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()

    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
    #joint_torques[:] = control_torques.value

    if i == 10000:
        quad_com_ctrl.set_bias()
        # print(quad_com_ctrl.com_imp_ctrl.displaySignals())
        print("completed switch")

    if i > 10000:
        ang_tau.recompute(i)
        print(i, ang_tau.value)


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


print "End of simulation"
