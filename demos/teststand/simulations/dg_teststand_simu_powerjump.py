## Simulation for power jump with teststand
## Author : Avadesh Meduri
## Date : 14/04/2019


from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

from dynamic_graph_manager.dg_tools import power_jump_control

#######################################################################################

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot


###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_teststand/config/teststand.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)


##############################################################################

des_weight_fff = constVector([0.0, 0.0, 0.8*9.81, 0.0, 0.0, 0.0], "des_weight_fff")
des_fff = constVector([0.0, 0.0, 2.0*9.81, 0.0, 0.0, 0.0], "des_fff")
des_pos_trigger = constVector([0.0, 0.0, -0.1, 0.0, 0.0, 0.0],"pos_des_trigger")
des_pos_air = constVector([0.0, 0.0, -0.25, 0.0, 0.0, 0.0],"pos_des_air")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

des_kp_ground = constVector([0.5, 0.0, 100.0, 0.0, 0.0, 0.0],"des_kp_ground")
des_kp_air = constVector([0.5, 0.0, 100.0, 0.0, 0.0, 0.0],"des_kp_air")
des_kd = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0],"des_kd_air")


##############################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)

leg_imp_ctrl.xyzpos_hip = hom2pos(leg_imp_ctrl.robot_dg.signal("pos_hip_" + leg_imp_ctrl.leg_name), "xyzpos_hip_" + leg_imp_ctrl.leg_name)
leg_imp_ctrl.xyzpos_foot = hom2pos(leg_imp_ctrl.robot_dg.signal("pos_foot_" + leg_imp_ctrl.leg_name), "xyzpos_foot_" + leg_imp_ctrl.leg_name)

leg_imp_ctrl.rel_pos_foot = subtract_vec_vec(leg_imp_ctrl.xyzpos_foot, leg_imp_ctrl.xyzpos_hip, "rel_pos_foot_" + leg_imp_ctrl.leg_name)
leg_imp_ctrl.rel_pos_foot = stack_two_vectors(leg_imp_ctrl.rel_pos_foot, constVector([0.0, 0.0, 0.0], 'stack_to_wrench_' + leg_imp_ctrl.leg_name), 3, 3)

power_jump_ctrl = power_jump_control("power_jump_ctrl")
plug(leg_imp_ctrl.rel_pos_foot, power_jump_ctrl.leg_length) #6d vector
plug(robot.device.contact_sensors, power_jump_ctrl.cnt_sensor)
plug(des_pos_trigger, power_jump_ctrl.leg_length_trigger)
plug(des_pos_air, power_jump_ctrl.leg_length_air)
plug(des_fff, power_jump_ctrl.des_fff)
plug(des_weight_fff, power_jump_ctrl.des_weight_fff)
plug(des_kp_ground, power_jump_ctrl.kp_ground)
plug(des_kp_air, power_jump_ctrl.kp_air)


des_pos = power_jump_ctrl.des_pos
des_force = power_jump_ctrl.des_force
des_kp = power_jump_ctrl.des_kp

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout


control_torques = leg_imp_ctrl.return_control_torques(des_kp, des_pos,
                                                    des_kd, des_vel, kf, des_force)



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

for i in range(40000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()

    #print(q, dq)

    des_pos.recompute(i)
    print(des_pos.value)

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
