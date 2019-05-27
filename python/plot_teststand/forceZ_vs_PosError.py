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

###############################################################################

name = str(sys.argv[1])
print("loading file : " + name)

sensors_data = RAI.sensors.SensorsData(name)
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

fig1, ax1 = plt.subplots(1,1, sharex = True)
ax1.plot(0.22-pos_error_height_sensor, Fz, color = "blue", label = "z_spring_behaviour")
ax1.set_xlim(0.0, 0.17)
ax1.set_ylim(0.0, 20.0)
ax1.legend()
ax1.set_xlabel("Vertical Foot Displacement [m]")
ax1.set_ylabel("Vertical Force [N]")
ax1.grid()

plt.show()
