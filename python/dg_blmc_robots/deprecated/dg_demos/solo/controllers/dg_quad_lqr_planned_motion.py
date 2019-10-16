## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control, quad_leg_impedance_controller
from leg_impedance_control.traj_generators import *

from dynamic_graph.sot.core.reader import Reader

from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dynamic_graph.sot.core.switch import SwitchVector



from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot


###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "./../../../../robots/robot_properties/robot_properties_quadruped/config/quadruped.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)

#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)
        assert False

#############################################################################
### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_pos_com = Reader('PositionComReader')
reader_vel_com = Reader('VelocityComReader')
reader_ori_com = Reader('OrientationComReader')
reader_ang_vel_com = Reader('AngVelComReader')
reader_forces = Reader('forces')
reader_lqr1 = Reader('lqr1')
reader_lqr2 = Reader('lqr2')
reader_lqr3 = Reader('lqr3')
reader_lqr4 = Reader('lqr3')



filename_pos = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_velocities_eff.dat"
filename_pos_com = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_com.dat"
filename_vel_com = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_com_vel.dat"
filename_ori_com = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_quaternion.dat"
filename_ang_vel_com = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_base_ang_velocities.dat"
filename_forces = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_forces.dat"
filename_lqr1 = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr1.dat"
filename_lqr2 = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr2.dat"
filename_lqr3 = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr3.dat"
filename_lqr4 = "./../../../../control/kino-dynamic-opt/momentumopt/demos/quadruped_lqr4.dat"


file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_pos_com)
file_exists(filename_vel_com)
file_exists(filename_ori_com)
file_exists(filename_ang_vel_com)
file_exists(filename_forces)
file_exists(filename_lqr1)
file_exists(filename_lqr2)
file_exists(filename_lqr3)
file_exists(filename_lqr4)


print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_pos_com.load(filename_pos_com)
reader_vel_com.load(filename_vel_com)
reader_ori_com.load(filename_ori_com)
reader_ang_vel_com.load(filename_ang_vel_com)
reader_forces.load(filename_forces)
reader_lqr1.load(filename_lqr1)
reader_lqr2.load(filename_lqr2)
reader_lqr3.load(filename_lqr3)
reader_lqr4.load(filename_lqr4)


# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_pos_com.selec.value = '1110'
reader_vel_com.selec.value = '1110'
reader_ori_com.selec.value = '11110'
reader_ang_vel_com.selec.value = '1110'
reader_forces.selec.value = '1111111111110'
reader_lqr1.selec.value = 39*'1'
reader_lqr2.selec.value = 39*'1'
reader_lqr3.selec.value = 39*'1'
reader_lqr4.selec.value = 39*'1'

###############################################################################
# com_pos_switch = SwitchVector("com_pos_switch")
# com_pos_switch.setSignalNumber(2)
# plug(reader_pos_com.vector, com_pos_switch.sin0)
# plug(constVector([0.0, 0.0, 0.2], "tmp"), com_pos_switch.sin1)
# com_pos_switch.selection.value = 0

###############################################################################

des_pos_com = reader_pos_com.vector
des_vel_com = reader_vel_com.vector
des_ori_com = reader_ori_com.vector
des_ang_vel_com = reader_ang_vel_com.vector

des_forces = reader_forces.vector
### sot_reader can read a vector of max size 40.
des_lqr1 = reader_lqr1.vector
des_lqr2 = reader_lqr2.vector
des_lqr3 = reader_lqr3.vector
des_lqr4 = reader_lqr4.vector


des_lqr = stack_two_vectors(des_lqr1, des_lqr2, 39, 39)
des_lqr = stack_two_vectors(des_lqr, des_lqr3, 78, 39)
des_lqr = stack_two_vectors(des_lqr, des_lqr4, 117, 39)


################################################################################

quad_com_ctrl = quad_com_control(robot, ViconClientEntity)
end_eff_lqr_forces = quad_com_ctrl.return_end_eff_lqr_tau(des_pos_com, des_vel_com, des_ori_com,  des_ang_vel_com, des_forces, des_lqr)

# ###############################################################################
#
def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_pos_com.rewind()
    reader_pos_com.vector.recompute(0)
    reader_vel_com.rewind()
    reader_vel_com.vector.recompute(0)
    reader_ori_com.rewind()
    reader_ori_com.vector.recompute(0)
    reader_ang_vel_com.rewind()
    reader_ang_vel_com.vector.recompute(0)
    reader_forces.rewind()
    reader_forces.vector.recompute(0)
    reader_lqr1.rewind()
    reader_lqr1.vector.recompute(0)
    reader_lqr2.rewind()
    reader_lqr2.vector.recompute(0)
    reader_lqr2.rewind()
    reader_lqr2.vector.recompute(0)
    reader_lqr3.rewind()
    reader_lqr3.vector.recompute(0)

# ################################################################################


##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 120.0
kp = add_kp.sout


kp = mul_double_vec_2(kp,  constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit"), "kp_split")

# kp = constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "kp_split")
kd = constVector([1., 0.0, 1., 0.0, 0.0, 0.0], "kd_split")

des_pos = reader_pos.vector
des_vel = reader_vel.vector

des_fff = end_eff_lqr_forces

add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)

####################################### data logging ########################################
quad_imp_ctrl.record_data()

quad_com_ctrl.record_data()

robot.add_trace("PositionReader", "vector")
robot.add_ros_and_trace("PositionReader", "vector")

robot.add_trace("VelocityReader", "vector")
robot.add_ros_and_trace("VelocityReader", "vector")


###############################################################################



######## robot simulation ################################################

from pinocchio.utils import zero

q = zero(8)
dq = zero(8)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(8)
joint_torques = zero(8)

q = zero(8)
dq = zero(8)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(8)
joint_torques = zero(8)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T


dt = 0.001#config.dt
motor_inertia = 0.045

for i in range(3000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()


    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
    #joint_torques[:] = control_torques.value

    des_fff.recompute(i)

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


print
print "End of simulation"
print "control_torques = ", robot.device.ctrl_joint_torques.value
