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

from plotter import *

################################################################################

def end_eff_forces(sensors_data):

    wbc_tau = []
    for i in range(12):
        wbc_tau.append(sensors_data.get_streams("data0/dg_quad_com_ctrl-wbctrl.dat[" + str(i) + "]"))

    com_torques = []
    for i in range(24):
        com_torques.append(sensors_data.get_streams("data0/dg_com_torques-sout.dat[" + str(i) + "]"))

    return np.asarray(wbc_tau), np.asarray(com_torques)


###############################################################################

name = str(sys.argv[1])
print("loading file : " + name)

sensors_data = read_data(name)

## to load RAI
load_file(name)
#
# assert False
#
# rel_pos_fl, rel_vel_fl, rel_pos_error_fl, rel_vel_error_fl = get_leg_data("fl", sensors_data)
# rel_pos_hl, rel_vel_hl, rel_pos_error_hl, rel_vel_error_hl = get_leg_data("hl", sensors_data)
# com_pos, com_vel = get_com_data(sensors_data)
# des_pos_fl, des_pos_fr, des_pos_hl, des_pos_hr, des_vel_fl, des_vel_fr, des_vel_hl, des_pos_hr = get_des_traj(sensors_data)
#
# s1, s2, s3 = get_slider_data(sensors_data)

# wbc_tau, com_torques = end_eff_forces(sensors_data)

####################################################################################

### Leg legnths des and actual
#
# fig1, ax1 = plt.subplots(4,1,sharex = True)
#
# ax1[0].plot(rel_pos_fl[: ,0], color = "red", label = "rel_pos_fl_x")
# ax1[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_fl_x")
# ax1[0].plot(rel_pos_fl[: ,1], color = "green", label = "rel_pos_fl_z")
# ax1[0].plot(des_pos_fl[: ,1], color = "blue", label = "des_pos_fl_z")
# ax1[0].legend()
# ax1[0].set_xlabel("millisec")
# ax1[0].set_ylabel("m")
# ax1[0].grid()
#
# ax1[1].plot(rel_pos_fr[: ,0], color = "red", label = "rel_posfrl_x")
# ax1[1].plot(des_pos_fr[:, 0], color = "black", label = "des_pos_fr_x")
# ax1[1].plot(rel_pos_fr[: ,1], color = "green", label = "rel_pos_fr_z")
# ax1[1].plot(des_pos_fr[: ,1], color = "blue", label = "des_pos_fr_z")
# ax1[1].legend()
# ax1[1].set_xlabel("millisec")
# ax1[1].set_ylabel("m")
# ax1[1].grid()
#
# ax1[2].plot(rel_pos_hl[: ,0], color = "red", label = "rel_pos_hl_x")
# ax1[2].plot(des_pos_hl[:, 0], color = "black", label = "des_pos_hl_x")
# ax1[2].plot(rel_pos_hl[: ,1], color = "green", label = "rel_pos_hl_z")
# ax1[2].plot(des_pos_hl[: ,1], color = "blue", label = "des_pos_hl_z")
# ax1[2].legend()
# ax1[2].set_xlabel("millisec")
# ax1[2].set_ylabel("m")
# ax1[2].grid()
#
# ax1[3].plot(rel_pos_hr[: ,0], color = "red", label = "rel_pos_hr_x")
# ax1[3].plot(des_pos_hr[:, 0], color = "black", label = "des_pos_hr_x")
# ax1[3].plot(rel_pos_hr[: ,1], color = "green", label = "rel_pos_hr_z")
# ax1[3].plot(des_pos_hr[: ,1], color = "blue", label = "des_pos_hr_z")
# ax1[3].legend()
# ax1[3].set_xlabel("millisec")
# ax1[3].set_ylabel("m")
# ax1[3].grid()
#
#
# ## relative velocities of leg des and actual
# fig2, ax2 = plt.subplots(4,1,sharex = True)
#
# ax2[0].plot(rel_vel_fl[: ,0], color = "red", label = "rel_vel_fl_x")
# ax2[0].plot(des_vel_fl[:, 0], color = "black", label = "des_vel_fl_x")
# ax2[0].plot(rel_vel_fl[: ,1], color = "green", label = "rel_vel_fl_z")
# ax2[0].plot(des_vel_fl[: ,1], color = "blue", label = "des_vel_fl_z")
# ax2[0].legend()
# ax2[0].set_xlabel("millisec")
# ax2[0].set_ylabel("m/s")
# ax2[0].grid()
#
# ax2[1].plot(rel_vel_fr[: ,0], color = "red", label = "rel_velfrl_x")
# ax2[1].plot(des_vel_fr[:, 0], color = "black", label = "des_vel_fr_x")
# ax2[1].plot(rel_vel_fr[: ,1], color = "green", label = "rel_vel_fr_z")
# ax2[1].plot(des_vel_fr[: ,1], color = "blue", label = "des_vel_fr_z")
# ax2[1].legend()
# ax2[1].set_xlabel("millisec")
# ax2[1].set_ylabel("m/s")
# ax2[1].grid()
#
# ax2[2].plot(rel_vel_hl[: ,0], color = "red", label = "rel_vel_hl_x")
# ax2[2].plot(des_vel_hl[:, 0], color = "black", label = "des_vel_hl_x")
# ax2[2].plot(rel_vel_hl[: ,1], color = "green", label = "rel_vel_hl_z")
# ax2[2].plot(des_vel_hl[: ,1], color = "blue", label = "des_vel_hl_z")
# ax2[2].legend()
# ax2[2].set_xlabel("millisec")
# ax2[2].set_ylabel("m/s")
# ax2[2].grid()
#
# ax2[3].plot(rel_vel_hr[: ,0], color = "red", label = "rel_vel_hr_x")
# ax2[3].plot(des_vel_hr[:, 0], color = "black", label = "des_vel_hr_x")
# ax2[3].plot(rel_vel_hr[: ,1], color = "green", label = "rel_vel_hr_z")
# ax2[3].plot(des_vel_hr[: ,1], color = "blue", label = "des_vel_hr_z")
# ax2[3].legend()
# ax2[3].set_xlabel("millisec")
# ax2[3].set_ylabel("m/s")
# ax2[3].grid()
#
#
# ### whole body control
# fig3, ax3 = plt.subplots(4,1,sharex = True)
# ax2[0].plot(wbc_tau[: ,0], color = "red", label = "wbc_fl_Fx")
# ax2[0].plot(com_torques[:, 0], color = "black", label = "com_tau_fl_Fx")
# ax2[0].plot(wbc_tau[: ,2], color = "green", label = "wbc_fl_Fz")
# ax2[0].plot(com_torques[: ,2], color = "blue", label = "com_tau_fl_Fz")
# ax2[0].legend()
# ax2[0].set_xlabel("millisec")
# ax2[0].set_ylabel("m/s")
# ax2[0].grid()
#
# ax2[1].plot(wbc_tau[: ,3], color = "red", label = "wbc_fr_Fx")
# ax2[1].plot(com_torques[:, 6], color = "black", label = "com_tau_fr_Fx")
# ax2[1].plot(wbc_tau[: ,5], color = "green", label = "wbc_fr_Fx")
# ax2[1].plot(com_torques[:, 8], color = "blue", label = "com_tau_fr_Fz")
# ax2[1].legend()
# ax2[1].set_xlabel("millisec")
# ax2[1].set_ylabel("m/s")
# ax2[1].grid()
#
# ax2[2].plot(wbc_tau[: ,6], color = "red", label = "rel_vel_hl_x")
# ax2[2].plot(com_torques[:, 12], color = "black", label = "des_vel_hl_x")
# ax2[2].plot(wbc_tau[: ,8], color = "green", label = "rel_vel_hl_z")
# ax2[2].plot(com_torques[:, 14], color = "blue", label = "des_vel_hl_z")
# ax2[2].legend()
# ax2[2].set_xlabel("millisec")
# ax2[2].set_ylabel("m/s")
# ax2[2].grid()
#
# ax2[3].plot(wbc_tau[: ,9], color = "red", label = "rel_vel_hr_x")
# ax2[3].plot(com_torques[:, 18], color = "black", label = "des_vel_hr_x")
# ax2[3].plot(wbc_tau[: ,11], color = "green", label = "rel_vel_hr_z")
# ax2[3].plot(com_torques[:, 20], color = "blue", label = "des_vel_hr_z")
# ax2[3].legend()
# ax2[3].set_xlabel("millisec")
# ax2[3].set_ylabel("m/s")
# ax2[3].grid()
#
#
# plt.show()
