import numpy as np
import RAI
from RAI.session import Session
from RAI.rai_launcher import launcher
import pkg_resources
from matplotlib import pyplot as plt

import pinocchio as pin
from pinocchio.rpy import matrixToRpy, npToTuple
from pinocchio.utils import XYZQUATToSe3

import sys, os

from numpy import math


################################################################################

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

def get_leg_data(leg_name, sensors_data):

    rel_pos_x = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_" + leg_name + "-sout.dat[0]"))
    rel_pos_z = np.array(sensors_data.get_streams("data0/dg_rel_pos_foot_" + leg_name + "-sout.dat[2]"))

    rel_vel_x = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_" + leg_name + "-sout.dat[0]"))
    rel_vel_z = np.array(sensors_data.get_streams("data0/dg_rel_vel_foot_" + leg_name + "-sout.dat[2]"))

    rel_pos_error_x = np.array(sensors_data.get_streams("data0/dg_pos_error_" + leg_name + "-sout.dat[0]"))
    rel_pos_error_z = np.array(sensors_data.get_streams("data0/dg_pos_error_" + leg_name + "-sout.dat[2]"))

    rel_vel_error_x = np.array(sensors_data.get_streams("data0/dg_vel_error_" + leg_name + "-sout.dat[0]"))
    rel_vel_error_z = np.array(sensors_data.get_streams("data0/dg_vel_error_" + leg_name + "-sout.dat[2]"))

    rel_pos = np.transpose(np.vstack((rel_pos_x, rel_pos_z)))
    rel_vel = np.transpose(np.vstack((rel_vel_x, rel_vel_z)))
    rel_pos_error = np.transpose(np.vstack((rel_pos_error_x, rel_pos_error_z)))
    rel_vel_error = np.transpose(np.vstack((rel_vel_error_x, rel_vel_error_z)))

    return rel_pos, rel_vel, rel_pos_error, rel_vel_error

def get_joint_torques(sensors_data):

    FL_u = []
    FR_u = []
    HL_u = []
    HR_u = []
    for i in range(4):
        HFE_u = np.array(sensors_data.get_streams("data0/dg_quadruped-joint_torques.dat[" + str(2*i) + "]"))
        KFE_u = np.array(sensors_data.get_streams("data0/dg_quadruped-joint_torques.dat[" + str(2*i + 1) + "]"))

        if i == 0:
            FL_u = np.transpose(np.vstack((HFE_u, KFE_u)))
        elif i == 1:
            FR_u = np.transpose(np.vstack((HFE_u, KFE_u)))
        elif i == 2:
            HL_u = np.transpose(np.vstack((HFE_u, KFE_u)))
        else:
            HR_u = np.transpose(np.vstack((HFE_u, KFE_u)))

    return FL_u, FR_u, HL_u, HR_u

def get_des_data(sensors_data):

    des_pos_fl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[0]")),
                            np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[2]")))))
    des_pos_fr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[6]")),
                            np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[8]")))))
    des_pos_hl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[12]")),
                            np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[14]")))))
    des_pos_hr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[18]")),
                            np.array(sensors_data.get_streams("data0/dg_pos_des-sout.dat[20]")))))

    des_vel_fl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[0]")),
                            np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[2]")))))
    des_vel_fr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[6]")),
                            np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[8]")))))
    des_vel_hl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[12]")),
                            np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[14]")))))
    des_vel_hr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[18]")),
                            np.array(sensors_data.get_streams("data0/dg_vel_des-sout.dat[20]")))))

    return des_pos_fl, des_vel_fl, des_pos_hl, des_vel_hl

def get_des_traj(sensors_data):

    des_pos_fl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[0]")),
                            np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[2]")))))
    des_pos_fr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[6]")),
                            np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[8]")))))
    des_pos_hl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[12]")),
                            np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[14]")))))
    des_pos_hr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[18]")),
                            np.array(sensors_data.get_streams("data0/dg_PositionReader-vector.dat[20]")))))

    des_vel_fl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[0]")),
                            np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[2]")))))
    des_vel_fr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[6]")),
                            np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[8]")))))
    des_vel_hl = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[12]")),
                            np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[14]")))))
    des_vel_hr = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[18]")),
                            np.array(sensors_data.get_streams("data0/dg_VelocityReader-vector.dat[20]")))))

    des_com = np.transpose(np.vstack((np.array(sensors_data.get_streams("data0/dg_ComReader-vector.dat[1]")),
                            np.array(sensors_data.get_streams("data0/dg_ComReader-vector.dat[3]")))))

    return des_pos_fl, des_pos_fr, des_pos_hl, des_pos_hr, des_vel_fl, des_vel_fr, des_vel_hl, des_pos_hr


def get_slider_data(sensors_data):
    s1 = np.multiply(160, np.array(sensors_data.get_streams("data0/dg_slider_fir_filter-sout.dat[0]")))
    s2 = np.multiply(0.07, np.array(sensors_data.get_streams("data0/dg_slider_fir_filter-sout.dat[0]")))
    s3 = np.multiply(1.5*np.pi, np.array(sensors_data.get_streams("data0/dg_slider_fir_filter-sout.dat[0]")))

    return s1, s2, s3

def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]


def get_com_data(sensors_data):
    com_x = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[0]"))
    com_y = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[1]"))
    com_z = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[2]"))
    com_q1 = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[3]"))
    com_q2 = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[4]"))
    com_q3 = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[5]"))
    com_q4 = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_position.dat[6]"))


    ang = []
    for i in range(len(com_x)):
        ang.append(quaternion_to_euler(com_q1[i], com_q2[i], com_q3[i], com_q4[i]))

    com_r = np.asarray(ang)

    com_rx = np.multiply(com_r[: ,2], 180.0/np.pi)
    com_ry = np.multiply(com_r[: ,1], 180.0/np.pi)
    com_rz = np.multiply(com_r[: ,0], 180.0/np.pi)

    com_xd = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_velocity_body.dat[0]"))
    com_zd = np.array(sensors_data.get_streams("data0/dg_solo-quadruped_velocity_body.dat[2]"))

    com_pos = np.transpose(np.vstack((com_x, com_z, com_rx, com_ry ,com_rz)))
    com_vel = np.transpose(np.vstack((com_xd, com_zd)))

    return com_pos, com_vel





################################################################################

def bias_data(data_stream):
    ## takes the av of the first 10 values and sets a bias

    av = np.average(data_stream[0:1200], axis=0)
    data_stream_biased = np.subtract(data_stream, av)

    return data_stream_biased


################################################################################

# name = str(sys.argv[1])
# print("loading file : " + name)
#
# sensors_data = read_data(name)

# load_file(name)
#
# assert False
#
# rel_pos_fl, rel_vel_fl, rel_pos_error_fl, rel_vel_error_fl = get_leg_data("fl", sensors_data)
# rel_pos_hl, rel_vel_hl, rel_pos_error_hl, rel_vel_error_hl = get_leg_data("hl", sensors_data)
# com_pos, com_vel = get_com_data(sensors_data)
# FL_u, FR_u, HL_u, HR_u = get_joint_torques(sensors_data)
# des_pos_fl, des_vel_fl, des_pos_hl, des_vel_hl = get_des_data(sensors_data)
# des_pos_fl, des_vel_fl, des_pos_hl, des_vel_hl, des_com = get_des_traj(sensors_data)

# s1, s2, s3 = get_slider_data(sensors_data)

# des_com = bias_data(des_com)
# com_pos = bias_data(com_pos)

###############################################################################

#### Position, Velocity and torques plot for fl and fr
#
# fig1, ax1 = plt.subplots(5,1,sharex = True)
#
# ax1[0].plot(rel_pos_fl[: ,0], color = "red", label = "rel_pos_fl_x")
# ax1[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_fl_x")
# ax1[0].legend()
# ax1[0].set_xlabel("millisec")
# ax1[0].set_ylabel("m")
# ax1[0].grid()
#
# ax1[1].plot(rel_pos_fl[: ,1], color = "red", label = "rel_pos_fl_z")
# ax1[1].plot(des_pos_fl[: ,1], color = "black", label = "des_pos_fl_z")
# ax1[1].legend()
# ax1[1].set_xlabel("millisec")
# ax1[1].set_ylabel("m")
# ax1[1].grid()
#
# ax1[2].plot(rel_vel_fl[: ,0], color = "red", label = "rel_vel_fl_x")
# ax1[2].plot(des_vel_fl[:, 0], color = "black", label = "des_vel_fl_x")
# ax1[2].legend()
# ax1[2].set_xlabel("millisec")
# ax1[2].set_ylabel("m/s")
# ax1[2].grid()
#
# ax1[3].plot(rel_vel_fl[: ,1], color = "red", label = "rel_vel_fl_z")
# ax1[3].plot(des_vel_fl[: ,1], color = "black", label = "des_vel_fl_z")
# ax1[3].legend()
# ax1[3].set_xlabel("millisec")
# ax1[3].set_ylabel("m/s")
# ax1[3].grid()
#
# ax1[4].plot(FL_u[: ,0], color = "red", label = "HFE_FL_u")
# ax1[4].plot(FL_u[: ,1], color = "black", label = "KFE_FL_u")
# ax1[4].legend()
# ax1[4].set_xlabel("millisec")
# ax1[4].set_ylabel("Nm")
# ax1[4].grid()
#
# #### Position, Velocity and torques plot for hl and hr

# fig4, ax4 = plt.subplots(5,1,sharex = True)
#
# ax4[0].plot(rel_pos_hl[: ,0], color = "red", label = "rel_pos_hl_x")
# ax4[0].plot(des_pos_hl[:, 0], color = "black", label = "des_pos_hl_x")
# ax4[0].legend()
# ax4[0].set_xlabel("millisec")
# ax4[0].set_ylabel("m")
# ax4[0].grid()
#
# ax4[1].plot(rel_pos_hl[: ,1], color = "red", label = "rel_pos_hl_z")
# ax4[1].plot(des_pos_hl[: ,1], color = "black", label = "des_pos_hl_z")
# ax4[1].legend()
# ax4[1].set_xlabel("millisec")
# ax4[1].set_ylabel("m")
# ax4[1].grid()
#
# ax4[2].plot(rel_vel_hl[: ,0], color = "red", label = "rel_vel_hl_x")
# ax4[2].plot(des_vel_hl[:, 0], color = "black", label = "des_vel_hl_x")
# ax4[2].legend()
# ax4[2].set_xlabel("millisec")
# ax4[2].set_ylabel("m/s")
# ax4[2].grid()
#
# ax4[3].plot(rel_vel_hl[: ,1], color = "red", label = "rel_vel_hl_z")
# ax4[3].plot(des_vel_hl[: ,1], color = "black", label = "des_vel_hl_z")
# ax4[3].legend()
# ax4[3].set_xlabel("millisec")
# ax4[3].set_ylabel("m/s")
# ax4[3].grid()
#
# ax4[4].plot(HL_u[: ,0], color = "red", label = "HFE_HL_u")
# ax4[4].plot(HL_u[: ,1], color = "black", label = "KFE_HL_u")
# ax4[4].legend()
# ax4[4].set_xlabel("millisec")
# ax4[4].set_ylabel("Nm")
# ax4[4].grid()
#



#### COM position , velocity plot
#
# fig2, ax2 = plt.subplots(4,1,sharex = True)
#
# ax2[0].plot(com_pos[: ,0], color = "red", label = "com_pos_x")
# # ax2[0].plot(des_com[:, 0], color = "black", label = "des_com_x")
# ax2[0].legend()
# ax2[0].set_xlabel("millisec")
# ax2[0].set_ylabel("m")
# ax2[0].grid()
#
# ax2[1].plot(com_pos[: ,1], color = "red", label = "com_pos_z")
# # ax2[1].plot(des_com[:, 1], color = "black", label = "des_com_z")
# ax2[1].legend()
# ax2[1].set_xlabel("millisec")
# ax2[1].set_ylabel("m")
# ax2[1].grid()
#
# ax2[2].plot(com_vel[: ,0], color = "red", label = "com_vel_x")
# #ax2[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_foot_x")
# ax2[2].legend()
# ax2[2].set_xlabel("millisec")
# ax2[2].set_ylabel("m/s")
# ax2[2].grid()
#
# ax2[3].plot(com_vel[: ,1], color = "red", label = "com_vel_z")
# #ax2[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_x")
# ax2[3].legend()
# ax2[3].set_xlabel("millisec")
# ax2[3].set_ylabel("m/s")
# ax2[3].grid()
#
# ### Com_rotation #################################################
#
# fig3, ax3 = plt.subplots(3,1,sharex = True)
#
# ax3[0].plot(com_pos[: ,2], color = "red", label = "com_rx")
# #ax3[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_x")
# ax3[0].legend()
# ax3[0].set_xlabel("millisec")
# ax3[0].set_ylabel("degree")
# ax3[0].grid()
#
# ax3[1].plot(com_pos[: ,3], color = "red", label = "com_ry")
# #ax3[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_x")
# ax3[1].legend()
# ax3[1].set_xlabel("millisec")
# ax3[1].set_ylabel("degree")
# ax3[1].grid()
#
# ax3[2].plot(com_pos[: ,4], color = "red", label = "com_rz")
# #ax3[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_x")
# ax3[2].legend()
# ax3[2].set_xlabel("millisec")
# ax3[2].set_ylabel("degree")
# ax3[2].grid()
#
# # ax3[3].plot(s1, color = "red", label = "p_gain")
# # ax3[3].plot(s2, color = "red", label = "amplitude")
# # ax3[3].plot(s3, color = "red", label = "phase")
# # #ax3[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_x")
# # ax3[3].legend()
# # ax3[3].set_xlabel("millisec")
# # ax3[3].set_ylabel("degree")
# # ax3[3].grid()
#
#
# # ###### Joint Torques #######################################################
# #
# # fig3, ax3 = plt.subplots(2,1, sharex = True)
# #
# # ax3[0].plot(FL_u[: ,0], color = "red", label = "HFE_FL_u")
# # ax3[0].plot(FR_u[: ,0], color = "black", label = "HFE_FR_u")
# # ax3[0].plot(HL_u[: ,0], color = "green", label = "HFE_HL_u")
# # ax3[0].plot(HR_u[: ,0], color = "orange", label = "HFE_HR_u")
# # ax3[0].legend()
# # ax3[0].set_xlabel("millisec")
# # ax3[0].set_ylabel("Nm")
# # ax3[0].grid()
# #
# # ax3[1].plot(FL_u[: ,1], color = "red", label = "KFE_FL_u")
# # ax3[1].plot(FR_u[: ,1], color = "black", label = "KFE_FR_u")
# # ax3[1].plot(HL_u[: ,1], color = "green", label = "KFE_HL_u")
# # ax3[1].plot(HR_u[: ,1], color = "orange", label = "KFE_HR_u")
# # ax3[1].legend()
# # ax3[1].set_xlabel("millisec")
# # ax3[1].set_ylabel("Nm")
# # ax3[1].grid()
# #
# plt.show()
