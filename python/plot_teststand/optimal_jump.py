### This file contains functions and plots to generate optimal jump
### Author : Avadesh meduri
### Date:  17th April



import numpy as np
from scipy.optimize import linprog
from matplotlib import pyplot as plt

tau_max = [3.5, 3.5]
l1 = 0.16
l2 = 0.16


def compute_jac(theta_1, theta_2):

    k1 = -l1*(np.sin(theta_1))
    k2 = -l2*(np.sin(theta_1 + theta_2))
    k3 = l1*(np.cos(theta_1))
    k4 = l2*(np.cos(theta_1 + theta_2))

    jacT = np.matrix([[k1 + k2, k3+k4], [k2, k4]])

    return jacT


def create_constrains(theta_1, theta_2):
    k1 = -l1*(np.sin(theta_1))
    k2 = -l2*(np.sin(theta_1 + theta_2))
    k3 = l1*(np.cos(theta_1))
    k4 = l2*(np.cos(theta_1 + theta_2))

    A = [[k3+k4],[k4]]
    b = [[tau_max[0]], [tau_max[1]]]

    return A, b

def inv_kin(x, z):

    k1 = np.sqrt(z) + np.sqrt(x) - np.sqrt(l1) - np.sqrt(l2)/2*l1*l2
    theta_2 = -np.arccos(k1)
    k2 = l2*np.sin(theta_2)
    k3 = l1 + l2*np.cos(theta_2)

    theta_1 = np.arctan(z/x) + np.arctan(k2/k3)

    return theta_1, theta_2

des_l = np.arange(0.01,0.31, 0.001)

##### optimization #######################

optimal_tau = []
c = [-1]
max_f = []
for l in des_l:
    theta_1, theta_2 = inv_kin(0, l)
    A,b = create_constrains(theta_1, theta_2)
    res = linprog(c, A_ub=A, b_ub=b)

    max_f.append(res.x[0])

    jact = compute_jac(theta_1, theta_2)
    f = [[0],[res.x[0]]]

    optimal_tau.append(np.matmul(jact, f))

optimal_tau = np.asarray(optimal_tau)

# print(optimal_tau)


# print(max_f)



fig1, ax1 = plt.subplots(2,1, sharex = True)
ax1[0].plot(des_l, optimal_tau[:, 0], color = "black", label = "optimal_hip_tau")
ax1[0].plot(des_l, optimal_tau[:, 1], color = "red", label = "optimal_knee_tau")
ax1[0].legend()
ax1[0].set_xlabel("m")
ax1[0].set_ylabel("nm")
ax1[0].grid()

ax1[1].plot(des_l, max_f)
ax1[1].set_xlabel("leg_length in meters")
ax1[1].set_ylabel("max_force in Newtons")
ax1[1].grid()

plt.show()


# plt.plot(des_l, optimal_tau[:,0])
