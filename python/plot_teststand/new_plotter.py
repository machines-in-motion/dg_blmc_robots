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
def load_file(name):

    # Sensors files
    sensors_filename = './data/' + name
    print(sensors_filename)
    # Set up views
    #views_data = {}
    #views_data['SingleView'] = {0: ['data0/y', 'data1/z'], 1: ['data1/y']}

    # Create and launch session
    my_session = Session(sensors_filename)
    my_session.launch()

def read_data(name):
    print("reading_data...")
    sensors_data = RAI.sensors.SensorsData("./data/" + name)
    print("completed reading data...")
    return sensors_data

def hopper_sensors_data(sensors_data):

    Fx = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
    Fy = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[1]"))
    Fz = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))

    F = np.transpose(np.vstack((Fx,Fy,Fz)))

    cnt_sensor = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-contact_sensors.dat[0]"))
    height_sensor = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-height_sensors.dat[0]"))

    HFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
    KFE_u = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))

    tau = np.transpose(np.vstack((HFE_u, KFE_u)))


    return F, cnt_sensor, height_sensor, tau


def hopper_foot_data(sensors_data):

    rel_pos_foot_x = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_hopper-sout.dat[0]"))
    rel_pos_foot_z = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_hopper-sout.dat[2]"))

    des_pos_x = np.array(sensors_data.get_streams("data0/dg_power_jump_ctrl-des_pos.dat[0]"))
    des_pos_z = np.array(sensors_data.get_streams("data0/dg_power_jump_ctrl-des_pos.dat[2]"))

    rel_vel_foot_x = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_hopper-sout.dat[0]"))
    rel_vel_foot_z = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_hopper-sout.dat[2]"))

    des_force_x = np.array(sensors_data.get_streams("data0/dg_power_jump_ctrl-des_force.dat[0]"))
    des_force_z = np.array(sensors_data.get_streams("data0/dg_power_jump_ctrl-des_force.dat[2]"))

    rel_pos_foot = np.transpose(np.vstack((rel_pos_foot_x, rel_pos_foot_z)))
    rel_vel_foot = np.transpose(np.vstack((rel_vel_foot_x, rel_vel_foot_z)))
    des_force = np.transpose(np.vstack((des_force_x, des_force_z)))
    des_pos = np.transpose(np.vstack((des_pos_x, des_pos_z)))

    des_vel = np.zeros((np.shape(des_pos)))

    return rel_pos_foot, rel_vel_foot, des_force, des_pos, des_vel

def hopper_joint_data(sensors_data):

    theta_1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[0]"))
    theta_2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_positions.dat[1]"))
    theta_d1 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[0]"))
    theta_d2 = np.array(sensors_data.get_streams("data0/dg_hopper_teststand-joint_velocities.dat[1]"))

    return theta_1, theta_2

##############################################################################

### functions to compute torques for forces read through ati sensor
def compute_torques(theta_1, theta_2, Fx, Fz):

    des_tau = []
    for i in range(len(theta_1)):

        fy = Fx[i]
        fx = Fz[i]

        F_vector = np.matrix([fx, fy])
        l1 = 0.16
        l2 = 0.16

        k1 = -l1*(np.sin(theta_1[i]))
        k2 = -l2*(np.sin(theta_1[i] + theta_2[i]))
        k3 = l1*(np.cos(theta_1[i]))
        k4 = l2*(np.cos(theta_1[i] + theta_2[i]))

        jacT = np.matrix([[k1 + k2, k3+k4], [k2, k4]])
        #print(jacT)
        torques = np.matmul(jacT, np.transpose(F_vector))
        des_tau.append(torques)


    return np.asarray(des_tau)

###############################################################################

name = str(sys.argv[1])
print("loading file : " + name)

sensors_data = read_data(name)


ati_f, cnt_sensor, ht_sensor, tau = hopper_sensors_data(sensors_data)
rel_pos_foot, rel_vel_foot, des_force, des_pos, des_vel = hopper_foot_data(sensors_data)
theta_1, theta_2 = hopper_joint_data(sensors_data)
des_tau = compute_torques(theta_1, theta_2, des_force[:, 0], des_force[:, 1])


###############################################################################


## plots for des_position and velocity vas actual values. and Hip and Knee torques
fig1, ax1 = plt.subplots(7,1, sharex = True)
ax1[0].plot(des_pos[:, 1], color = "black", label = "des_pos_z")
ax1[0].plot(rel_pos_foot[:, 1], color = "red", label = "actual_foot_pos_z")
ax1[0].legend()
ax1[0].set_xlabel("millisec")
ax1[0].set_ylabel("m")
ax1[0].grid()

ax1[1].plot(ht_sensor, color = "black", label = "ht_sensor")
ax1[1].legend()
ax1[1].set_xlabel("millisec")
ax1[1].set_ylabel("m")
ax1[1].grid()

ax1[2].plot(ati_f[: ,0], color = "black", label = "Fx")
ax1[2].plot(des_force[: ,0], color = "red", label = "des_fx")
ax1[2].legend()
ax1[2].set_xlabel("millisec")
ax1[2].set_ylabel("N")
ax1[2].grid()

ax1[3].plot(ati_f[: ,2], color = "black", label = "Fz")
ax1[3].plot(des_force[: ,1], color = "red", label = "des_Fz")
ax1[3].legend()
ax1[3].set_xlabel("millisec")
ax1[3].set_ylabel("N")
ax1[3].grid()

ax1[4].plot(cnt_sensor, color = "black", label = "cnt_sensor")
ax1[4].legend()
ax1[4].set_xlabel("millisec")
ax1[4].set_ylabel("No-metric")
ax1[4].grid()

ax1[5].plot(tau[: ,0], color = "red", label = "HFE_u")
ax1[5].plot(des_tau[: ,0], color = "green", label = "des_HFE_u")
ax1[5].set_xlabel("millisec")
ax1[5].set_ylabel("Nm")
ax1[5].legend()
ax1[5].grid()

ax1[6].plot(tau[: ,1], color = "red", label = "KFE_u")
ax1[6].plot(des_tau[: ,1], color = "green", label = "des_KFE_u")
ax1[6].set_xlabel("millisec")
ax1[6].set_ylabel("Nm")
ax1[6].legend()
ax1[6].grid()

plt.show()
