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


#
# Plot parameters
#
font = {'family' : 'normal',
        # 'weight' : 'bold',
        'size'   : 22}
matplotlib.rc('font', **font)


########### Testing ###############################################################

def get_data(rai_data, start_value, end_value):

  Fx = np.array(rai_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[0]"))
  Fz = np.array(rai_data.get_streams("data0/dg_hopper_teststand-ati_force.dat[2]"))
  pos_error_x = np.array(rai_data.get_streams("data0/dg_pos_error_hopper-sout.dat[0]"))
  pos_error_z = np.array(rai_data.get_streams("data0/dg_pos_error_hopper-sout.dat[2]"))
  pos_error_height_sensor = np.array(rai_data.get_streams("data0/dg_stiffness_measurement_height_sensor-sout.dat[0]"))
  HFE_u = np.array(rai_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[0]"))
  KFE_u = np.array(rai_data.get_streams("data0/dg_hopper_teststand-joint_torques.dat[1]"))
  k_value_x = np.divide(Fx, pos_error_x)
  k_value_z = np.divide(Fz, pos_error_z)
  pos_error_height_sensor = 0.22 - pos_error_height_sensor

  Fx = Fx[start_value:end_value]
  Fz = Fz[start_value:end_value]
  pos_error_x = pos_error_x[start_value:end_value]
  pos_error_z = pos_error_z[start_value:end_value]
  k_value_x = k_value_x[start_value:end_value]
  k_value_z = k_value_z[start_value:end_value]
  HFE_u = HFE_u[start_value:end_value]
  KFE_u = KFE_u[start_value:end_value]
  pos_error_height_sensor = pos_error_height_sensor[start_value:end_value]

  return Fx, Fz, pos_error_x, pos_error_z, pos_error_height_sensor, HFE_u, KFE_u, k_value_x, k_value_z

###############################################################################
nb_file = sys.argc - 1

sensors_data = []
data = []
Fx_1 = []
Fz_1 = []
pos_error_x_1 = []
pos_error_z_1 = []
pos_error_height_sensor_1 = []
HFE_u_1 = []
KFE_u_1 = []
k_value_x_1 = []
k_value_z_1 = []

for i in range(nb_file):
    name = sys.argv[i]
    print("loading file : " + name)
    sensors_data.append(RAI.sensors.SensorsData(name))
    data.append(sensors_data1.get_all_streams())


#### limit values
start_value = 0
end_value = -1

(Fx_1,
 Fz_1,
 pos_error_x_1,
 pos_error_z_1,
 pos_error_height_sensor_1,
 HFE_u_1,
 KFE_u_1,
 k_value_x_1,
 k_value_z_1) = get_data(sensors_data1, 0, -1)

(Fx_2,
 Fz_2,
 pos_error_x_2,
 pos_error_z_2,
 pos_error_height_sensor_2,
 HFE_u_2,
 KFE_u_2,
 k_value_x_2,
 k_value_z_2) = get_data(sensors_data2, 0, -1)

fig1, ax1 = plt.subplots(1,1, sharex = True)
ax1.plot(pos_error_height_sensor_1, Fz_1, color = "black", label = "knee left")
ax1.plot(pos_error_height_sensor_2, Fz_2, color = "red", label = "knee_right")
ax1.set_xlim(0.0, 0.17)
ax1.set_ylim(0.0, 20.0)
ax1.legend()
ax1.set_xlabel("Vertical Foot Displacement [m]")
ax1.set_ylabel("Vertical Force [N]")
ax1.grid()

plt.show()
