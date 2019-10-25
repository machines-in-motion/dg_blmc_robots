## This is code to track desired trajectories from the planner using the whole body controller
## The gains have are obtained from the centroidal LQR code inside kino-dyno opt
## and leg impedance

##Author : Avadesh Meduri
#Date : 8/03/19


from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values

from dynamic_graph.sot.core.switch import SwitchVector

##########################################################################################
import dg_blmc_robots
from dg_blmc_robots.solo.solo_bullet import get_quadruped_robot, ViconClientEntity

import pinocchio as se3
from pinocchio.utils import zero


# Get the robot corresponding to the quadruped.
robot = get_quadruped_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.2
q[1] = 0.0
q[2] = 0.22
q[6] = 1.
q[7] = 0.8
q[8] = -1.6
q[9] = 0.8
q[10] = -1.6
q[11] = -0.8
q[12] = 1.6
q[13] = -0.8
q[14] = 1.6
# Update the initial state of the robot.
robot.reset_state(q, dq)


################################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

from dynamic_graph.sot.core.reader import Reader

reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_abs_vel = Reader('AbsVelReader') # absolute velocity of the end_effector
reader_pos_com = Reader('PositionComReader')
reader_vel_com = Reader('VelocityComReader')
reader_fff_com = Reader('FeedForwardForceComReader')
reader_ori_com = Reader('OrientationComReader')
reader_ang_vel_com = Reader('AngVelComReader')
reader_fft_com = Reader('FeedForwardMomentsComReader')
reader_cnt_plan = Reader("CntPlan")

reader_lqr1 = Reader("CentLQR1") ## Lqr split because there is a limit in SOT reader for size of read array
reader_lqr2 = Reader("CentLQR2")

filename_pos = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_velocities_eff.dat"
filename_abs_vel = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_velocities_abs.dat"
filename_cnt_plan = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_contact_activation.dat"

filename_pos_com = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_com.dat"
filename_vel_com = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_com_vel.dat"
filename_fff_com =  "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_centroidal_forces.dat"
filename_ori_com = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_quaternion.dat"
filename_ang_vel_com = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_base_ang_velocities.dat"
filename_fft_com =  "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_centroidal_moments.dat"

filename_lqr1 = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_centroidal_gains1.dat"
filename_lqr2 = "../../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_centroidal_gains2.dat"

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_abs_vel)
file_exists(filename_cnt_plan)
file_exists(filename_pos_com)
file_exists(filename_vel_com)
file_exists(filename_fff_com)
file_exists(filename_ori_com)
file_exists(filename_ang_vel_com)
file_exists(filename_fft_com)
file_exists(filename_lqr1)
file_exists(filename_lqr2)


print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_abs_vel.load(filename_abs_vel)
reader_cnt_plan.load(filename_cnt_plan)
reader_pos_com.load(filename_pos_com)
reader_vel_com.load(filename_vel_com)
reader_fff_com.load(filename_fff_com)
reader_ori_com.load(filename_ori_com)
reader_ang_vel_com.load(filename_ang_vel_com)
reader_fft_com.load(filename_fft_com)

reader_lqr1.load(filename_lqr1)
reader_lqr2.load(filename_lqr2)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_abs_vel.selec.value = '1111111111110'
reader_cnt_plan.selec.value = '11110'
reader_pos_com.selec.value = '1110'
reader_vel_com.selec.value = '1110'
reader_ori_com.selec.value = '11110'
reader_fff_com.selec.value = '1110'
reader_ang_vel_com.selec.value = '1110'
reader_fft_com.selec.value = '1110'

reader_lqr1.selec.value = 39*'1'
reader_lqr1.selec.value = 39*'1'



###############################################################################
def gen_r_matrix(rx, ry, rz):
    R = np.matrix([[0, -rz, ry],
                   [rz, 0, -rx],
                   [-ry, rx, 0]])
    return R


def gen_v_matrix(vx, vy, vz):
    V = np.matrix([[vx, 0, 0],
                   [0, vy, 0],
                   [0, 0, vz]])
    return V

###############################################################################


I = np.matrix([[1.0 , 0.0 , 0.0],
               [0.0 , 1.0 , 0.0],
               [0.0 , 0.0 , 1.0]])

r_fl = gen_r_matrix(0.2, 0.15, 0.0)
r_fr = gen_r_matrix(0.2, -0.15, 0.0)
r_hl = gen_r_matrix(-0.2, 0.15, 0.0)
r_hr = gen_r_matrix(-0.2, -0.15, 0.0)

v_fl = gen_v_matrix(0.0, 0.0, 0.0)
v_fr = gen_v_matrix(0.0, 0.0, 0.0)
v_hl = gen_v_matrix(0.0, 0.0, 0.0)
v_hr = gen_v_matrix(0.0, 0.0, 0.0)

zm = np.zeros((3,3))

py_ce = np.block([[I, I, I, I, -I, zm, zm, zm, zm, zm ],
                  [r_fl, r_fr, r_hl, r_hr, zm, -I, zm, zm, zm, zm],
                  [v_fl, zm, zm ,zm, zm, zm, -I, zm, zm, zm],
                  [zm, v_fr, zm ,zm, zm, zm, zm, -I, zm, zm],
                  [zm, zm, v_hl ,zm, zm, zm, zm, zm, -I, zm],
                  [zm, zm, zm ,v_hr, zm, zm, zm, zm, zm, -I]])

py_ce = np.asarray(py_ce)

w1 = 10.0
w2 = 160000.0
w3 = 100.0
py_hess = np.zeros((30,30))
np.fill_diagonal(py_hess, w1)


py_hess[12][12] = w3
py_hess[13][13] = w3
py_hess[14][14] = w3
py_hess[15][15] = w3
py_hess[16][16] = w3
py_hess[17][17] = w3

py_hess[18][18] = w2
py_hess[19][19] = w2
py_hess[20][20] = w2
py_hess[21][21] = w2
py_hess[22][22] = w2
py_hess[23][23] = w2
py_hess[24][24] = w2
py_hess[25][25] = w2
py_hess[26][26] = w2
py_hess[27][27] = w2
py_hess[28][28] = w2
py_hess[29][29] = w2


py_reg = np.zeros((30,30))
np.fill_diagonal(py_reg, 0.0001)

hess = constMatrix(py_hess, "hess")
reg = constMatrix(py_reg, "regularizer")
g0 = zero_vec(30, "g0")
ce = constMatrix(py_ce, "ce")
ci = constMatrix(np.zeros((18,30)), "ci")
ci0 = zero_vec(18, "ci0")

###############################################################################


###############################################################################

des_pos_com = reader_pos_com.vector
des_vel_com = reader_vel_com.vector
des_abs_vel = reader_abs_vel.vector
des_cnt_plan = reader_cnt_plan.vector
des_fff_com = reader_fff_com.vector
des_ori_com = reader_ori_com.vector
des_ang_vel_com = reader_ang_vel_com.vector
des_fft_com = reader_fft_com.vector
des_lqr = stack_two_vectors(reader_lqr1.vector, reader_lqr2.vector, 39,39)

################################################################################

quad_com_ctrl = quad_com_control(robot, ViconClientEntity, "solo")
lqr_com_force = quad_com_ctrl.return_lqr_tau(des_pos_com, des_vel_com, des_ori_com, des_ang_vel_com, des_fff_com, des_fft_com, des_lqr)
lctrl = selec_vector(lqr_com_force, 0,3, "lqr_lctrl")
actrl = selec_vector(lqr_com_force, 3,6, "lqr_actrl")
com_torques = quad_com_ctrl.return_com_torques(lctrl, actrl, des_abs_vel, hess, g0, ce, ci, ci0, reg, des_cnt_plan)


###############################################################################
kp = constVector([100.0, 0.0, 100.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([5.0, 0.0, 5.0, 0.0, 0.0, 0.0], "kd_split")

des_pos = reader_pos.vector
des_vel = reader_vel.vector

des_fff = com_torques

add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 1.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

###############################################################################
quad_com_ctrl.record_data()

###############################################################################
robot.run(10000, 1./60.)
