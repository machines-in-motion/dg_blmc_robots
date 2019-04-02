#impedance controller implementation for COM (used for quadruped)
#Author : Avadesh Meduri
#Date : 25/03/19


from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control
from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dynamic_graph_manager.dg_tools import ComImpedanceControl
from dynamic_graph.sot.core.switch import SwitchVector

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "/home/ameduri/devel/workspace/src/catkin/robots/robot_properties/robot_properties_teststand/config/teststand.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)


################################################################################

kp = constVector([1.0, 1.0, 1.0], "kp")
kd = constVector([1.0, 1.0, 1.0], "kd")
des_pos = constVector([0.0, 0.0, 0.0], "des_pos")
des_vel = constVector([0.0, 0.0, 0.0], "des_vel")
des_fff = constVector([0.0, 0.0, 0.0], "des_fff")
###############################################################################
EntityName = "quad_com_ctrl"
vicon_ip = '10.32.3.16:801'
client_name = "vicon_client"
host_name_quadruped = vicon_ip
robot_vicon_name = "quadruped"

vicon_client = ViconClientEntity(client_name)
vicon_client.connect_to_vicon(host_name_quadruped)
vicon_client.displaySignals()

vicon_client.add_object_to_track("{}/{}".format(robot_vicon_name, robot_vicon_name))

robot.add_ros_and_trace(client_name, robot_vicon_name + "_position")
robot.add_ros_and_trace(client_name, robot_vicon_name + "_velocity_body")
robot.add_ros_and_trace(client_name, robot_vicon_name + "_velocity_world")

base_pos_xyz = selec_vector(vicon_client.signal(robot_vicon_name + "_position"),
                                0, 3, "selec_xyz")
base_vel_xyz = selec_vector(vicon_client.signal(robot_vicon_name + "_velocity_body"),
                                0, 3, "selec_dxyz")


################################################################################

com_imp_ctrl = ComImpedanceControl(EntityName)

control_switch_pos = SwitchVector("control_switch_pos")
control_switch_pos.setSignalNumber(2) # we want to switch between 2 signals
plug(zero_vec(3,"zero"), control_switch_pos.sin0)
plug(com_imp_ctrl.set_pos_bias, control_switch_pos.sin1)
control_switch_pos.selection.value = 0

# biased_base_pos_xyz = subtract_vec_vec(base_pos_xyz, control_switch_pos.sout, "bias_pos")

plug(base_pos_xyz, com_imp_ctrl.position)
plug(base_vel_xyz, com_imp_ctrl.velocity)

plug(kp, com_imp_ctrl.Kp)
plug(kd, com_imp_ctrl.Kd)
plug(des_pos, com_imp_ctrl.des_pos)
plug(des_vel, com_imp_ctrl.des_vel)
plug(des_fff, com_imp_ctrl.des_fff)




###############################################################################

robot.add_trace("quad_com_ctrl", "tau")
robot.add_ros_and_trace("quad_com_ctrl", "tau")

robot.add_trace("control_switch_pos", "sout")
robot.add_ros_and_trace("control_switch_pos", "sout")


#
# robot.add_trace("quad_com_ctrl", "setbias")
# robot.add_ros_and_trace("quad_com_ctrl", "setbias")


######## robot simulation ################################################

from pinocchio.utils import zero

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

print(q, dq)

dt = 0.001#config.dt
motor_inertia = 0.045

for i in range(40000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()


    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
    #joint_torques[:] = control_torques.value

    if i == 10000:
        control_switch_pos.selection.value = 1
        print("completed switch")

    if i > 10000:
        print(i, control_switch_pos.sout.value)


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
