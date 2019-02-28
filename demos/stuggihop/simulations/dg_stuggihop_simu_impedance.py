import numpy as np

import pinocchio as se3
from pinocchio.utils import zero

import py_dg_blmc_robots
from py_dg_blmc_robots.stuggihop import get_stuggihop_robot

from dynamic_graph import plug
from dynamic_graph.sot.core.operator import Stack_of_vector

from py_dg_blmc_robots.stuggihop import get_stuggihop_robot
from py_dg_blmc_robots.stuggihop import StuggihopConfig
import dynamic_graph.sot.dynamics_pinocchio as dyn_pin

from leg_impedance_control.utils import hom2pos, stack_zero, constVector, mul_double_vec, mul_mat_vec
from leg_impedance_control.utils import compute_pos_diff, stack_two_vectors, Mat_transpose

from dynamic_graph.sot.core import Selec_of_vector
from dynamic_graph.sot.core.operator import Multiply_double_vector, Add_of_double


######## DEFS 
print("Begin")
robot_py = StuggihopConfig.buildRobotWrapper()
robot_dg = dyn_pin.DynamicPinocchio('hopper')
robot_dg.setData(robot_py.data)
robot_dg.setModel(robot_py.model)

robot_dg.createJacobianEndEffWorld('jac_contact', 'contact')
robot_dg.createPosition('pos_hip', 'HFE')
robot_dg.createPosition('pos_foot', 'END')


# ######## SETUP ########
print("Starting Setup")
# Get the robot corresponding to the stuggi hop.
robot = get_stuggihop_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.25
q[2] = 0.8
q[3] = -1.6

# Update the initial state of the robot.
robot.set_gravity((0,0,-9.81))
robot.reset_state(q, dq)

# Plug the position and velocity from the robot into the robot_dg.
robot_dg.acceleration.value = 4 * (0.0, )


plug(stack_two_vectors(constVector([0.0, 0.0],'add_base_joint_position'), 
    robot.device.signal('joint_positions'), 2, 2), robot_dg.position)
plug(stack_two_vectors(constVector([0.0, 0.0], 'add_base_joint_velocities'), 
    robot.device.signal('joint_velocities'), 2, 2), robot_dg.velocity)
print("Setup finished")

######## DEFS FOR IMPEDANCE ########

def compute_impedance_torques(jac, errors_vec, names = ["","","",""]):
    '''
    TODO: add a selector matrix. Currently the selection is not generic at all.
    '''
    if len(names) != 4:
        print("ERROR: compute_impedance_torques requires 4 unique names for entities. " 
               +"You can also leave this empty, and it will generate names.")
    if names[0] == "":
        print("WARNING: it is recommended to give names to every entity yoursef.")

    ##Transpose tau = JacT*(errors)
    ## errors = [x - xdes, y-ydes]T
    jac_T = Mat_transpose(jac, names[0]) # TODO: Is this safe?
    ## multiplying negative
    errors_vec = mul_double_vec(-1.0, errors_vec, names[1])
    control_torques = mul_mat_vec(jac_T, errors_vec, 
                                        names[2])
    # selecting only 2nd and 3rd element in torques as element one 
    # represents base acceleration
    select_mat = Selec_of_vector(names[3])
    select_mat.selec(2,4) # This is not generic... should have a selector matrix
    plug(control_torques, select_mat.signal('sin'))
    return select_mat.signal('sout')

def impedance_controller(robot_dg, kv, des_pos):
    ##### Single leg impedance controller

    xyzpos_hip = hom2pos(robot_dg.pos_hip, "xyzpos_hip")
    xyzpos_foot = hom2pos(robot_dg.pos_foot, "xyzpos_foot")
    # relative foot position to hip
    rel_pos_foot = compute_pos_diff(xyzpos_foot, xyzpos_hip, "rel_pos_foot")

    jac = robot_dg.jac_contact
    pos_error = compute_pos_diff(rel_pos_foot, des_pos, "pos_error")
    # Stacking rotations after cartesian pos_error, to make this into a twist,
    # since the Jacobian has been taken w.r.t. to a twist (pos, rot)
    pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0,0.0],
                                'stack_to_twist'), 3, 3)

    mul_double_vec_op = Multiply_double_vector("gain_multiplication")
    plug(kv, mul_double_vec_op.sin1)
    plug(pos_error, mul_double_vec_op.sin2)
    pos_error_with_gains = mul_double_vec_op.sout

    control_torques = compute_impedance_torques(jac, pos_error_with_gains,
        ["jacTranspose","neg_op","compute_control_torques","impedance_torques"])
    # control_torques = 0
    return control_torques

######## CONTROL ########

# For making gain input dynamic through terminal
# TODO: use a const instead....
add = Add_of_double('mult')
add.sin1.value = 0
### Change this value for different gains
add.sin2.value = 120.0
kv = add.sout # virtual spring stiffness

# in end-effector cartesian frame (x-y-z)
des_pos = constVector([0.0, 0.0, -0.2],"pos_des")

## Impdance control implementation

control_torques = impedance_controller(robot_dg, kv, des_pos)

plug(control_torques, robot.device.ctrl_joint_torques)

# # # Setup the control graph to track the desired joint positions.
# # # pd = PDController("controller_1")
# # # setup the gains
# # pd.kv.value = 2 * (5.,)
# # pd.Kd.value = 2 * (0.1,)
# # # define the desired position and set teh desired velocity 0.
# # pd.desired_position.value = (np.pi/4.0, -np.pi/4)
# # pd.desired_velocity.value = 2 * (0.,)
# # # plug the desired quantity signals in the pd controller.
# # plug(robot.device.joint_positions, pd.position)
# # plug(robot.device.joint_velocities, pd.velocity)

# # # plug the ouput of the pd controller to the robot motor torques
# # plug(pd.control, robot.device.ctrl_joint_torques)


##### RUNNING SIMULATION
robot.run(500000, 1./60.)

######## robot simulation ################################################

# from pinocchio.utils import zero

# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)

# q = zero(2)
# dq = zero(2)
# q_out = q.T.tolist()[0]
# dq_out = dq.T.tolist()[0]
# ddq = zero(2)
# joint_torques = zero(2)

# q[:] = np.asmatrix(robot.device.joint_positions.value).T
# dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

# print(q, dq)

# dt = 0.001#config.dt
# motor_inertia = 0.045

# for i in range(40000):
#     # fill the sensors
#     robot.device.joint_positions.value = q.T.tolist()[0][:]
#     robot.device.joint_velocities.value = dq.T.tolist()[0][:]

#     # "Execute the dynamic graph"
#     robot.device.execute_graph()


#     joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
#     #joint_torques[:] = control_torques.value

#     #print(q, dq)
#     pos_des_fl_fr.recompute(i)
#     print(pos_des_fl_fr.value)

#     # integrate the configuration from the computed torques
#     #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
#     #dq = (dq + dt * joint_torques / motor_inertia)
#     q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
#     dq = (dq + dt * 0.01 / motor_inertia)


#     if (i % 1000) == 0:
#         # print "qref =     ", robot.pid_control.pose_controller.qRef.value
#         # print ("q =        ",
#         #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
#         # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
#         # print "currents = ", robot.device.ctrl_joint_torques.value
#         pass


# print
# print "End of simulation"
# print "control_torques = ", robot.device.ctrl_joint_torques.value


# raw_input("Press Enter to continue...")