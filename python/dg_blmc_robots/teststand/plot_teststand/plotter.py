import numpy as np
import RAI
import pkg_resources
import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt

import sys, os

from numpy import math

import pinocchio as se3


##############################################################################
### functions to compute torques for forces read through ati sensor
def compute_torques(theta_1, theta_2, Fx, Fz):

    fy = Fx
    fx = -Fz

    F_vector = np.matrix([fx, fy])
    l1 = 0.16
    l2 = 0.16

    k1 = -l1*(np.sin(theta_1))
    k2 = -l2*(np.sin(theta_1 + theta_2))
    k3 = l1*(np.cos(theta_1))
    k4 = l2*(np.cos(theta_1 + theta_2))

    jacT = np.matrix([[k1 + k2, k3+k4], [k2, k4]])
    #print(jacT)
    torques = np.matmul(jacT, np.transpose(F_vector))

    return torques


########### Testing ###############################################################


#### read data file
if_check = int(sys.argv[1])

#print(compute_torques(np.pi/4.0, -np.pi/2.0, 0, 2.0))

###############################################################################

if if_check == 1:

    name = str(sys.argv[2])
    print("loading file : " + name)

    sensors_data = RAI.sensors.SensorsData("./data/" + name)
    data = sensors_data.get_all_streams()

    #data = sensors_data.get_all_streams()
    ################ data0/dg_pos_error-sout.dat[]
    ################ data0/dg_rel_pos_foot-sout.dat[]

    Fx = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
    Fz = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
    pos_error_x = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[0]"))
    pos_error_z = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
    pos_error_height_sensor = np.array(sensors_data.get_streams("data0/dg_stiffness_measurement_height_sensor-sout.dat[0]"))
    HFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
    KFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))


    k_value_x = np.divide(Fx, pos_error_x)
    k_value_z = np.divide(Fz, pos_error_z)

    ### (29,17707)
    #print(np.shape(data[0]))
    #### limit values
    start_value = 0
    end_value = -1

    Fx = Fx[start_value:end_value]
    Fz = Fz[start_value:end_value]
    pos_error_x = pos_error_x[start_value:end_value]
    pos_error_z = pos_error_z[start_value:end_value]
    k_value_x = k_value_x[start_value:end_value]
    k_value_z = k_value_z[start_value:end_value]
    HFE_u = HFE_u[start_value:end_value]
    KFE_u = KFE_u[start_value:end_value]
    pos_error_height_sensor = pos_error_height_sensor[start_value:end_value]

    fig1, ax1 = plt.subplots(2,2,sharex = True)
    ax1[0,0].plot(np.subtract(Fx,0), color= "red", label = "Fx")
    ax1[0,0].plot(np.subtract(Fz,0), color = "blue", label = "Fz")
    ax1[0,0].legend()
    ax1[0,0].set_xlabel("millisec")
    ax1[0,0].set_ylabel("N")
    ax1[0,0].grid()

    ax1[1,0].plot(pos_error_x, color = "red", label = "error_x")
    ax1[1,0].plot(pos_error_z, color = "black", label = "error_z")
    ax1[1,0].legend()
    ax1[1,0].set_xlabel("millisec")
    ax1[1,0].set_ylabel("m")
    ax1[1,0].grid()

    #ax1[0,1].plot(k_value_x, color = "red", label = "kx")
    ax1[0,1].plot(k_value_z, color = "black", label = "kz")
    ax1[0,1].legend()
    ax1[0,1].set_xlabel("millisec")
    ax1[0,1].set_ylabel("spring_constant")
    ax1[0,1].set_ylim(0,300)
    ax1[0,1].grid()

    #ax1[1,1].plot(pos_error_x, color = "red", label = "error_x")
    ax1[1,1].plot(-HFE_u, color = "red", label = "HFE_u")
    ax1[1,1].plot(-KFE_u, color = "green", label = "KFE_u")
    ax1[1,1].set_xlabel("millisec")
    ax1[1,1].set_ylabel("Nm")
    ax1[1,1].legend()
    ax1[1,1].grid()

    fig2, ax2 = plt.subplots(3,1)

    ax2[0].plot(pos_error_z, Fz, color = "black", label = "z_spring_behaviour")
    ax2[0].plot(0.22-pos_error_height_sensor, Fz, color = "red", label = "z_spring_behaviour")
    ax2[0].legend()
    ax2[0].set_xlabel("m")
    ax2[0].set_xlim(-0.15, 0.15)
    ax2[0].set_ylabel("N")
    ax2[0].grid()

    ax2[1].plot(0.22-pos_error_height_sensor, Fz, color = "red", label = "z_spring_behaviour")
    ax2[1].legend()
    ax2[1].set_xlabel("m")
    ax2[1].set_ylabel("N")
    ax2[1].grid()
    
    ax2[2].plot(pos_error_x, Fx, color = "red", label = "x_spring_behaviour")
    ax2[2].legend()
    ax2[2].set_xlabel("m")
    ax2[2].set_ylabel("N")
    ax2[2].grid()

    plt.show()

if if_check == 2:

    name1 = str(sys.argv[2])
    name2 = str(sys.argv[3])
    sensors_data = RAI.sensors.SensorsData("./data/" + name1)
    Fz_1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
    pos_error_z_1 = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
    k_value_z_1 = np.divide(Fz_1, pos_error_z_1)

    sensors_data = RAI.sensors.SensorsData("./data/" + name2)
    Fz_2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
    pos_error_z_2 = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
    k_value_z_2 = np.divide(Fz_2, pos_error_z_2)

    start_value = 4000
    end_value = -2000

    Fz_1 = Fz_1[start_value:end_value]
    Fz_2 = Fz_2[start_value:end_value]
    pos_error_z_1 = pos_error_z_1[start_value:end_value]
    pos_error_z_2 = pos_error_z_2[start_value:end_value]

    plt.figure()
    plt.plot(pos_error_z_1, Fz_1, color = "red", label = "spring_constant - Knee_to_right")
    plt.plot(pos_error_z_2, Fz_2, color = "black", label = "spring_constant - Knee_to_left")
    plt.xlabel("Displacement (m) ")
    plt.ylabel("Force (N)")
    #plt.xlim(0,0.15)
    #plt.ylim(0,15)
    plt.legend()
    plt.grid()
    plt.show()

if if_check == 3:

    ## plots for friction estimation at joints
    name = str(sys.argv[2])
    print("loading file : " + name)

    sensors_data = RAI.sensors.SensorsData("./data/" + name)
    data = sensors_data.get_all_streams()

    #data = sensors_data.get_all_streams()
    ################ data0/dg_pos_errorhopper-sout.dat[]
    ################ data0/dg_rel_pos_foot-sout.dat[]

    Fx = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
    Fz = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
    pos_error_x = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[0]"))
    pos_error_z = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
    HFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
    KFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))
    theta_1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[0]"))
    theta_2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[1]"))
    theta_d1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[0]"))
    theta_d2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[1]"))

    motor_HFE = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-motor_positions.dat[0]"))
    motor_KFE = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-motor_positions.dat[1]"))

    k_value_x = np.divide(Fx, pos_error_x)
    k_value_z = np.divide(Fz, pos_error_z)

    ### (29,17707)
    #print(np.shape(data[0]))
    #### limit values
    start_value = 0
    end_value = -10

    Fx = Fx[start_value:end_value]
    Fz = Fz[start_value:end_value]
    pos_error_x = pos_error_x[start_value:end_value]
    pos_error_z = pos_error_z[start_value:end_value]
    k_value_x = k_value_x[start_value:end_value]
    k_value_z = k_value_z[start_value:end_value]
    HFE_u = HFE_u[start_value:end_value]
    KFE_u = KFE_u[start_value:end_value]
    theta_1 = theta_1[start_value:end_value]
    theta_2 = theta_2[start_value:end_value]
    theta_d1 = theta_d1[start_value:end_value]
    theta_d2 = theta_d2[start_value:end_value]


    friction_torques_1 = []
    friction_torques_2 = []
    for i in range(len(theta_1)):
        friction_torques = compute_torques(theta_1[i], theta_2[i], Fx[i]-0.7, Fz[i])
        friction_torques_1.append(float(friction_torques[0]))
        friction_torques_2.append(float(friction_torques[1]))

    hip_fric_vel_ratio = np.divide(np.multiply(friction_torques_1, -20.0), theta_d1)
    knee_fric_vel_ratio = np.divide(np.multiply(friction_torques_2, 20.0), theta_d2)


    # plt.figure()
    # plt.plot(pos_error_z, friction_torques_1, color = "red", label = "")
    # plt.xlabel("Displacement (m) ")
    # plt.ylabel("Torque (Nm)")
    # #plt.xlim(0,0.15)
    # #plt.ylim(0,15)
    # plt.legend()
    # plt.grid()


    fig1, ax1 = plt.subplots(2,1, sharex = True)
    ax1[0].plot(theta_d1, color = "black", label = "hip_joint_velocities")
    #ax1[0].plot(np.multiply(friction_torques_1, -20.0), color = "red", label = "friction_hip_torques * 20")
    ax1[0].legend()
    ax1[0].set_xlabel("millisec")
    ax1[0].set_ylabel("rad/sec")
    ax1[0].grid()

    ax1[1].plot(theta_d2, color = "red", label = "knee_joint_velocities")
    # ax2[0].plot(np.multiply(friction_torques_2, 20.0), color = "black", label = "friction_knee_torques * 20")
    #ax2[0].plot(theta_2, color = "black", label = "knee_joint_position")
    ax1[1].legend()
    ax1[1].set_xlabel("millisec")
    ax1[1].set_ylabel("rad/sec")
    ax1[1].grid()

    # ax1[0].plot(motor_HFE, color = "black", label = "hip_motor_position")
    # #ax1[1].plot(friction_torques_1, color = "red", label = "friction_hip_torques")
    # ax1[1].legend()
    # ax1[1].set_xlabel("millisec")
    # ax1[1].set_ylabel("rad")
    # ax1[1].grid()

    # ax1[2].plot(hip_fric_vel_ratio, color = "red", label = "friction_hip_ratio")
    # ax1[2].legend()
    # ax1[2].set_xlabel("millisec")
    # ax1[2].set_ylabel("Nmsec/rad")
    # ax1[2].set_ylim(-15, 10)

    #ax1[2].grid()


    fig2, ax2 = plt.subplots(2,1, sharex = True)
    ax2[0].plot(theta_d2, color = "red", label = "knee_joint_velocities")
    # ax2[0].plot(np.multiply(friction_torques_2, 20.0), color = "black", label = "friction_knee_torques * 20")
    #ax2[0].plot(theta_2, color = "black", label = "knee_joint_position")
    ax2[0].legend()
    ax2[0].set_xlabel("millisec")
    ax2[0].set_ylabel("rad/sec")
    ax2[0].grid()

    ax2[1].plot(motor_KFE, color = "black", label = "knee_motor_position")
    # ax2[1].plot(friction_torques_2, color = "black", label = "friction_knee_torques")
    ax2[1].legend()
    ax2[1].set_xlabel("millisec")
    ax2[1].set_ylabel("rad")
    ax2[1].grid()

    # ax2[2].plot(knee_fric_vel_ratio, color = "black", label = "friction_knee_ratio")
    # ax2[2].legend()
    # ax2[2].set_xlabel("millisec")
    # ax2[2].set_ylabel("Nmsec/rad")
    # ax2[2].set_ylim(-15, 10)

#    ax2[2].grid()

    plt.show()

if if_check == 4:

    ## plots postion tracking experiments
    ## to understand why position tracking is not smooth
    name = str(sys.argv[2])
    print("loading file : " + name)

    sensors_data = RAI.sensors.SensorsData("./data/" + name)
    data = sensors_data.get_all_streams()

    #data = sensors_data.get_all_streams()
    ################ data0/dg_pos_errorhopper-sout.dat[]
    ################ data0/dg_rel_pos_foot-sout.dat[]

    pos_error_x = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[0]"))
    pos_error_z = np.array(sensors_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))

    rel_pos_foot_x = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_hopper-sout.dat[0]"))
    rel_pos_foot_z = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_hopper-sout.dat[2]"))

    des_pos_x = np.array(sensors_data.get_streams("data0/dg_des_leg_length_pos-sout.dat[0]"))
    des_pos_z = np.array(sensors_data.get_streams("data0/dg_des_leg_length_pos-sout.dat[2]"))

    des_vel_x = np.array(sensors_data.get_streams("data0/dg_des_leg_length_vel-sout.dat[0]"))
    des_vel_z = np.array(sensors_data.get_streams("data0/dg_des_leg_length_vel-sout.dat[2]"))

    rel_vel_foot_x = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_hopper-sout.dat[0]"))
    rel_vel_foot_z = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_hopper-sout.dat[2]"))

    HFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
    KFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))

    theta_1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[0]"))
    theta_2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[1]"))

    theta_d1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[0]"))
    theta_d2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[1]"))


    ## to evaluate coggin with modulo function
    motor_HFE = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-motor_positions.dat[0]"))
    motor_KFE = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-motor_positions.dat[1]"))

    #########################################################################################################
    ### (29,17707)
    #print(np.shape(data[0]))
    #### limit values
    start_value = 0
    end_value = -1000

    pos_error_x = pos_error_x[start_value:end_value]
    pos_error_z = pos_error_z[start_value:end_value]
    des_pos_x = des_pos_x[start_value:end_value]
    des_pos_z = des_pos_z[start_value:end_value]
    des_vel_x = des_vel_x[start_value:end_value]
    des_vel_z = des_vel_z[start_value:end_value]
    HFE_u = HFE_u[start_value:end_value]
    KFE_u = KFE_u[start_value:end_value]
    motor_HFE = motor_HFE[start_value:end_value]
    motor_KFE = motor_KFE[start_value:end_value]
    theta_1 = theta_1[start_value:end_value]
    theta_2 = theta_2[start_value:end_value]
    theta_d1 = theta_d1[start_value:end_value]
    theta_d2 = theta_d2[start_value:end_value]
    rel_pos_foot_x = rel_pos_foot_x[start_value:end_value]
    rel_pos_foot_z = rel_pos_foot_z[start_value:end_value]


    ################# computing position error offline
    min_len = int(min(len(rel_pos_foot_z), len(des_pos_z)))
    print(min_len)
    pos_err_z = np.subtract(rel_pos_foot_z[0:min_len], des_pos_z[0:min_len])
    pos_err_x = np.subtract(rel_pos_foot_x[0:min_len], des_pos_x[0:min_len])

    motor_HFE_modulo = np.mod(motor_HFE, 5.0*np.pi/180.0)
    motor_KFE_modulo = np.mod(motor_KFE, 5.0*np.pi/180.0)


    motor_HFE = motor_HFE[0:min_len]
    motor_KFE = motor_KFE[0:min_len]
    motor_HFE_modulo = motor_HFE_modulo[0:min_len]
    motor_KFE_modulo = motor_KFE_modulo[0:min_len]

    ## plots for des_position and velocity vas actual values. and Hip and Knee torques
    fig1, ax1 = plt.subplots(4,1, sharex = True)
    ax1[0].plot(des_pos_z, color = "black", label = "des_pos_z")
    ax1[0].plot(rel_pos_foot_z, color = "red", label = "actual_foot_pos_z")
    ax1[0].legend()
    ax1[0].set_xlabel("millisec")
    ax1[0].set_ylabel("m")
    ax1[0].grid()

    ax1[1].plot(des_pos_x, color = "black", label = "des_pos_x")
    ax1[1].plot(rel_pos_foot_x, color = "red", label = "actual_foot_pos_x")
    ax1[1].legend()
    ax1[1].set_xlabel("millisec")
    ax1[1].set_ylabel("m")
    ax1[1].grid()

    ax1[2].plot(rel_vel_foot_z, color = "black", label = "rel_vel_foot_z")
    ax1[2].plot(des_vel_z, color = "red", label = "des_vel_foot_z")
    ax1[2].legend()
    ax1[2].set_xlabel("millisec")
    ax1[2].set_ylabel("m/s")
    ax1[2].grid()

    ax1[3].plot(-HFE_u, color = "red", label = "HFE_u")
    ax1[3].plot(-KFE_u, color = "green", label = "KFE_u")
    ax1[3].set_xlabel("millisec")
    ax1[3].set_ylabel("Nm")
    ax1[3].legend()
    ax1[3].grid()

    ### plots of rel and actual velocity
    fig2, ax2 = plt.subplots(2,1, sharex = True)
    ax2[0].plot(rel_vel_foot_z, color = "black", label = "rel_vel_foot_z")
    ax2[0].plot(des_vel_z, color = "red", label = "des_vel_foot_z")
    ax2[0].legend()
    ax2[0].set_xlabel("millisec")
    ax2[0].set_ylabel("m/s")
    ax2[0].grid()

    ax2[1].plot(rel_vel_foot_x, color = "black", label = "rel_vel_foot_x")
    ax2[1].plot(des_vel_x, color = "red", label = "des_vel_foot_x")
    ax2[1].legend()
    ax2[1].set_xlabel("millisec")
    ax2[1].set_ylabel("m/s")
    ax2[1].grid()


    ## plots of motor positions with modulo
    fig3, ax3 = plt.subplots(3,1, sharex=True)

    ax3[0].plot(-HFE_u, color = "red", label = "HFE_u")
    ax3[0].plot(-KFE_u, color = "green", label = "KFE_u")
    #ax3.plot(theta_2, -KFE_u, color = "green", label = "KFE_u")
    ax3[0].set_xlabel("millisec")
    ax3[0].set_ylabel("Nm")
    ax3[0].legend()
    ax3[0].grid()

    ax3[1].plot(np.multiply(motor_HFE_modulo, 180.0/np.pi), color = "red", label = "motor_HFE_modulo")
    #ax3.plot(theta_2, -KFE_u, color = "green", label = "KFE_u")
    ax3[1].set_xlabel("millisec")
    ax3[1].set_ylabel("degree")
    ax3[1].legend()
    ax3[1].grid()

    ax3[2].plot(np.multiply(motor_KFE_modulo, 180.0/np.pi), color = "green", label = "motor_KFE_modulo")
    #ax3.plot(theta_2, -KFE_u, color = "green", label = "KFE_u")
    ax3[2].set_xlabel("millisec")
    ax3[2].set_ylabel("degree")
    ax3[2].legend()
    ax3[2].grid()

    # #ax3[3].plot(rel_pos_foot_z, color = "red", label = "actual_foot_pos_z")
    # ax3[3].plot(np.multiply(motor_HFE, 180.0/np.pi), color = "red", label = "motor_HFE")
    #
    # ax3[3].legend()
    # ax3[3].set_xlabel("millisec")
    # ax3[3].set_ylabel("degree")
    # ax3[3].grid()
    #
    # ax3[4].plot(np.multiply(motor_HFE_modulo, 180.0/np.pi), color = "black", label = "motor_HFE_modulo")
    # ax3[4].legend()
    # ax3[4].set_xlabel("millisec")
    # ax3[4].set_ylabel("degree")
    # ax3[4].grid()

    plt.show()


if if_check == 5:
    ## Force estimation with contact sensors

    name = str(sys.argv[2])
    print("loading file : " + name)

    sensors_data = RAI.sensors.SensorsData("./data/" + name)
    data = sensors_data.get_all_streams()


    Fx = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
    Fz = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))

    HFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
    KFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))

    est_Fx = np.array(sensors_data.get_streams("data0/dg_est_f_hopper-sout.dat[0]"))
    est_Fz = np.array(sensors_data.get_streams("data0/dg_est_f_hopper-sout.dat[2]"))

    cnt_sensor = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-contact_sensors.dat[0]"))

    height_sensor = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-height_sensors.dat[0]"))


    ### Fusing forces with contact sensor for more accurate contact sensing

    cnt_confidence = np.subtract(1,cnt_sensor)
    est_Fx_with_cnt_sensor = np.multiply(cnt_confidence, est_Fx)
    est_Fz_with_cnt_sensor = np.multiply(cnt_confidence, est_Fz)

    fig1, ax1 = plt.subplots(3,1, sharex=True)
    ax1[0].plot(Fx, color = "black", label = "Fx")
    ax1[0].plot(est_Fx, color = "red", label = "estimated_fx")
    ax1[0].legend()
    ax1[0].set_xlabel("millisec")
    ax1[0].set_ylabel("N")
    ax1[0].grid()

    ax1[1].plot(Fz, color = "black", label = "Fz")
    ax1[1].plot(est_Fz, color = "red", label = "estimated_Fz")
    ax1[1].legend()
    ax1[1].set_xlabel("millisec")
    ax1[1].set_ylabel("N")
    ax1[1].grid()

    ax1[2].plot(cnt_sensor, color = "black", label = "cnt_sensor")
    ax1[2].legend()
    ax1[2].set_xlabel("millisec")
    ax1[2].set_ylabel("No-metric")
    ax1[2].grid()

    fig2, ax2 = plt.subplots(3,1, sharex=True)
    ax2[0].plot(Fx, color = "black", label = "Fx")
    ax2[0].plot(est_Fx_with_cnt_sensor, color = "red", label = "fused_estimated_fx")
    ax2[0].legend()
    ax2[0].set_xlabel("millisec")
    ax2[0].set_ylabel("N")
    ax2[0].grid()

    ax2[1].plot(Fz, color = "black", label = "Fz")
    ax2[1].plot(est_Fz_with_cnt_sensor, color = "red", label = "fused_estimated_Fz")
    ax2[1].legend()
    ax2[1].set_xlabel("millisec")
    ax2[1].set_ylabel("N")
    ax2[1].grid()

    ax2[2].plot(height_sensor, color = "black", label = "height")
    ax2[2].legend()
    ax2[2].set_xlabel("millisec")
    ax2[2].set_ylabel("m")
    ax2[2].grid()



    plt.show()
