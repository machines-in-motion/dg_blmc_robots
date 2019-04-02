#impedance controller implementation for COM (used for quadruped)
#Author : Avadesh Meduri
#Date : 25/03/19


from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_com_control
from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dynamic_graph_manager.dg_tools import ComImpedanceControl
from dynamic_graph.sot.core.switch import SwitchVector
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
