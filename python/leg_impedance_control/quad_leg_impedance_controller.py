## impedance controller implementation on test stand
## Author : Avadesh Meduri
## Date : 1/03/19

########################### Imports ###########################################

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
# from leg_impedance_control.traj_generators import mul_double_vec_2
from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dynamic_graph_manager.dg_tools import ComImpedanceControl
from dynamic_graph.sot.core.switch import SwitchVector


from py_robot_properties_quadruped.config import QuadrupedConfig


###############################################################################


class quad_leg_impedance_controller():
    def __init__(self, robot):
        self.robot = robot

        self.imp_ctrl_leg_fl = leg_impedance_controller("fl")
        self.imp_ctrl_leg_fr = leg_impedance_controller("fr")
        self.imp_ctrl_leg_hl = leg_impedance_controller("hl")
        self.imp_ctrl_leg_hr = leg_impedance_controller("hr")

    def return_control_torques(self, kp, des_pos, kd=None, des_vel=None, kf = None, fff=None):
        """
        Input : Kp - proportional gain (double)
              : des_pos - 1*12 vector of desired_position in current time step (size : 1*24 )
              : Kd - derivative gain (double)
              : des_vel - 1*12 vector of desired_velocity in current time step (size : 1*24)
              : fff - Feed forward force (size : 1*24)
        """
        self.joint_positions = self.robot.device.signal("joint_positions")
        self.joint_velocities = self.robot.device.signal("joint_velocities")

        ### For FL #############################################################

        joint_positions_fl = selec_vector(self.joint_positions, 0, 2, 'position_slicer_fl')
        joint_velocities_fl = selec_vector(self.joint_velocities, 0, 2, 'velocity_slicer_fl')

        plug(stack_zero((joint_positions_fl),"add_base_joint_position_fl"), self.imp_ctrl_leg_fl.robot_dg.position)
        plug(stack_zero((joint_velocities_fl), "add_base_joint_velocity_fl"), self.imp_ctrl_leg_fl.robot_dg.velocity )

        des_pos_fl = selec_vector(des_pos, 0, 6, 'des_position_slicer_fl')
        if des_vel is not None:
            des_vel_fl = selec_vector(des_vel, 0, 6, 'des_veolcity_slicer_fl')
        else:
            des_vel_fl = None

        if fff is not None:
            fff_fl = selec_vector(fff, 0, 6, 'fff_slicer_fl')
        else:
            fff_fl = None

        control_torques_fl = self.imp_ctrl_leg_fl.return_control_torques(kp, des_pos_fl, kd, des_vel_fl, kf, fff_fl)

        ## For FR ##############################################################

        joint_positions_fr = selec_vector(self.joint_positions, 2, 4, 'position_slicer_fr')
        joint_velocities_fr = selec_vector(self.joint_velocities, 2, 4, 'velocity_slicer_fr')

        plug(stack_zero((joint_positions_fr),"add_base_joint_position_fr"), self.imp_ctrl_leg_fr.robot_dg.position)
        plug(stack_zero((joint_velocities_fr), "add_base_joint_velocity_fr"), self.imp_ctrl_leg_fr.robot_dg.velocity )

        des_pos_fr = selec_vector(des_pos, 6, 12, 'des_position_slicer_fr')
        if des_vel is not None:
            des_vel_fr = selec_vector(des_vel, 6, 12, 'des_veolcity_slicer_fr')
        else:
            des_vel_fr = None

        if fff is not None:
            fff_fr = selec_vector(fff, 6, 12, 'fff_slicer_fr')
        else:
            fff_fr = None

        control_torques_fr = self.imp_ctrl_leg_fr.return_control_torques(kp, des_pos_fr, kd, des_vel_fr, kf, fff_fr)
        # control_torques_fr = zero_vec(2, "zero_torque_fr")

        ### For HL #############################################################

        joint_positions_hl = selec_vector(self.joint_positions, 4, 6, 'position_slicer_hl')
        joint_velocities_hl = selec_vector(self.joint_velocities, 4, 6, 'velocity_slicer_hl')

        plug(stack_zero((joint_positions_hl),"add_base_joint_position_hl"), self.imp_ctrl_leg_hl.robot_dg.position)
        plug(stack_zero((joint_velocities_hl), "add_base_joint_velocity_hl"), self.imp_ctrl_leg_hl.robot_dg.velocity )

        des_pos_hl = selec_vector(des_pos, 12, 18, 'des_position_slicer_hl')
        if des_vel is not None:
            des_vel_hl = selec_vector(des_vel, 12, 18, 'des_veolcity_slicer_hl')
        else:
            des_vel_hl = None

        if fff is not None:
            fff_hl = selec_vector(fff, 12, 18, 'fff_slicer_hl')
        else:
            fff_hl = None

        control_torques_hl = self.imp_ctrl_leg_hl.return_control_torques(kp, des_pos_hl, kd, des_vel_hl, kf, fff_hl)

        ## For HR ##############################################################

        joint_positions_hr = selec_vector(self.joint_positions, 6, 8, 'position_slicer_hr')
        joint_velocities_hr = selec_vector(self.joint_velocities, 6, 8, 'velocity_slicer_hr')

        plug(stack_zero((joint_positions_hr),"add_base_joint_position_hr"), self.imp_ctrl_leg_hr.robot_dg.position)
        plug(stack_zero((joint_velocities_hr), "add_base_joint_velocity_hr"), self.imp_ctrl_leg_hr.robot_dg.velocity )

        des_pos_hr = selec_vector(des_pos, 18, 24, 'des_position_slicer_hr')
        if des_vel is not None:
            des_vel_hr = selec_vector(des_vel, 18, 24, 'des_veolcity_slicer_hr')
        else:
            des_vel_hr = None

        if fff is not None:
            fff_hr = selec_vector(fff, 18, 24, 'fff_slicer_hr')
        else:
            fff_hr = None

        control_torques_hr = self.imp_ctrl_leg_hr.return_control_torques(kp, des_pos_hr, kd, des_vel_hr, kf, fff_hr)
        # control_torques_hr = zero_vec(2, "zero_torque_hr")

        ####################### Stacking torques of each leg into one vector #####

        control_torques_fl_fr = stack_two_vectors(control_torques_fl, control_torques_fr, 2, 2)
        control_torques_hl_hr = stack_two_vectors(control_torques_hl, control_torques_hr, 2, 2)

        control_torques = stack_two_vectors(control_torques_fl_fr,control_torques_hl_hr, 4, 4)

        return control_torques

    def record_data(self, record_vicon = False):
        self.imp_ctrl_leg_fl.record_data(self.robot)
        self.imp_ctrl_leg_fr.record_data(self.robot)
        self.imp_ctrl_leg_hl.record_data(self.robot)
        self.imp_ctrl_leg_hr.record_data(self.robot)


class quad_com_control():
    def __init__(self, robot, client_name = "vicon_client" , vicon_ip = '10.32.3.16:801', EntityName = "quad_com_ctrl"):

        self.robot = robot
        self.EntityName = EntityName
        self.client_name = client_name
        self.host_name_quadruped = vicon_ip
        self.robot_vicon_name = "quadruped"

        self.vicon_client = ViconClientEntity(self.client_name)
        self.vicon_client.connect_to_vicon(self.host_name_quadruped)
        self.vicon_client.displaySignals()

        self.vicon_client.add_object_to_track("{}/{}".format(self.robot_vicon_name, self.robot_vicon_name))

        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_position")
        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_velocity_body")
        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_velocity_world")

        self.com_imp_ctrl = ComImpedanceControl(EntityName)

    def compute_torques(self, Kp, des_pos, Kd, des_vel, des_fff):


        self.base_pos_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_position"),
                                        0, 3, "base_pos")
        self.base_vel_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"),
                                        0, 3, "base_vel")
        # self.base_ang_vel_xyz =  selec_vector(self.vicon_client.signal(robot_vicon_name + "_velocity_body"),
        #                                 3, 6, "selec_ang_dxyz")

        plug(self.base_pos_xyz, self.com_imp_ctrl.position)
        plug(self.base_vel_xyz, self.com_imp_ctrl.velocity)
        # plug(self.base_ang_vel_xyz, self.com_imp_ctrl.angvel)

        plug(Kp, self.com_imp_ctrl.Kp)
        plug(Kd, self.com_imp_ctrl.Kd)
        plug(des_pos, self.com_imp_ctrl.des_pos)
        plug(des_vel, self.com_imp_ctrl.des_vel)
        plug(des_fff, self.com_imp_ctrl.des_fff)
        ### mass in all direction (double to vec returns zero)
        ## TODO : Check if there is dynamicgraph::double
        self.com_imp_ctrl.mass.value = [2.2, 2.2, 2.2]
        # plug(inertia, self.com_imp_ctrl.inertia)

        self.control_switch_pos = SwitchVector("control_switch_pos")
        self.control_switch_pos.setSignalNumber(2) # we want to switch between 2 signals
        plug(zero_vec(3,"zero_pos"), self.control_switch_pos.sin0)
        plug(self.com_imp_ctrl.set_pos_bias, self.control_switch_pos.sin1)
        self.control_switch_pos.selection.value = 0

        self.control_switch_vel = SwitchVector("control_switch_vel")
        self.control_switch_vel.setSignalNumber(2) # we want to switch between 2 signals
        plug(zero_vec(3,"zero_vel"), self.control_switch_vel.sin0)
        plug(self.com_imp_ctrl.set_vel_bias, self.control_switch_vel.sin1)
        plug(self.control_switch_pos.selection, self.control_switch_vel.selection )


        self.biased_base_pos_xyz = subtract_vec_vec(self.base_pos_xyz,
                                        self.control_switch_pos.sout, "biased_pos")
        self.biased_base_vel_xyz = subtract_vec_vec(self.base_vel_xyz,
                                        self.control_switch_vel.sout, "biased_vel")

        plug(self.biased_base_pos_xyz, self.com_imp_ctrl.biased_pos)
        plug(self.biased_base_vel_xyz, self.com_imp_ctrl.biased_vel)

        self.torques = self.com_imp_ctrl.tau

        return self.torques

    def set_bias(self):
        self.control_switch_pos.selection.value = 1

    def threshold_cnt_sensor(self):
        plug(self.robot.device.contact_sensors, self.com_imp_ctrl.cnt_sensor)
        return self.com_imp_ctrl.thr_cnt_sensor

    def convert_cnt_value_to_3d(self, cnt_sensor, start_index, end_index, entityName):
        ## Converts each element of contact sensor to 3d to allow vec_vec multiplication
        selec_cnt_value = selec_vector(cnt_sensor, start_index, end_index, entityName)
        cnt_value_2d = stack_two_vectors(selec_cnt_value, selec_cnt_value, 1, 1)
        cnt_value_3d = stack_two_vectors(cnt_value_2d, selec_cnt_value, 2, 1)

        return cnt_value_3d

    def return_com_torques(self, torques):
        ### This divides forces by four to get force per leg and fuses with contact sensor
        thr_cnt_sensor = self.threshold_cnt_sensor()

        torques_fl = mul_double_vec(0.25, torques, self.EntityName + "torques_fl")
        fl_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 0, 1, "fl_cnt_3d")
        fused_torques_fl = mul_vec_vec(fl_cnt_value, torques_fl, "fused_fl")

        torques_fr = mul_double_vec(0.25, torques, self.EntityName + "torques_fr")
        fr_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 1, 2, "fr_cnt_3d")
        fused_torques_fr = mul_vec_vec(fr_cnt_value, torques_fr, "fused_fr")

        torques_hl = mul_double_vec(0.25, torques, self.EntityName + "torques_hl")
        hl_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 2, 3, "hl_cnt_3d")
        fused_torques_hl = mul_vec_vec(hl_cnt_value, torques_hl, "fused_hl")

        torques_hr = mul_double_vec(0.25, torques, self.EntityName + "torques_hr")
        hr_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 3, 4, "hr_cnt_3d")
        fused_torques_hr = mul_vec_vec(hr_cnt_value, torques_hr, "fused_hr")

        fused_torques_fl_6d = stack_two_vectors(fused_torques_fl,
                                            zero_vec(3, "stack_fl_tau"), 3, 3)
        fused_torques_fr_6d = stack_two_vectors(fused_torques_fr,
                                            zero_vec(3, "stack_fr_tau"), 3, 3)
        fused_torques_hl_6d = stack_two_vectors(fused_torques_hl,
                                            zero_vec(3, "stack_hl_tau"), 3, 3)
        fused_torques_hr_6d = stack_two_vectors(fused_torques_hr,
                                            zero_vec(3, "stack_hr_tau"), 3, 3)

        fused_torques_fl_fr = stack_two_vectors(fused_torques_fl_6d,
                                                    fused_torques_fr_6d, 6, 6)
        fused_torques_hl_hr = stack_two_vectors(fused_torques_hl_6d,
                                                    fused_torques_hr_6d, 6, 6)

        com_torques = stack_two_vectors(fused_torques_fl_fr, fused_torques_hl_hr
                                                                    , 12, 12)
        ## hack to allow tracking of torques
        com_torques = add_vec_vec(com_torques, zero_vec(24, "add_com_tau"), "com_torques")

        return com_torques

    def record_data(self):
        self.robot.add_trace(self.EntityName, "tau")
        self.robot.add_ros_and_trace(self.EntityName, "tau")

        self.robot.add_trace(self.EntityName, "thr_cnt_sensor")
        self.robot.add_ros_and_trace(self.EntityName, "thr_cnt_sensor")

        self.robot.add_trace(self.EntityName + "torques_fl", "sout")
        self.robot.add_ros_and_trace(self.EntityName + "torques_fl", "sout")

        self.robot.add_trace("com_torques", "sout")
        self.robot.add_ros_and_trace("com_torques", "sout")

        self.robot.add_trace("control_switch_vel", "sout")
        self.robot.add_ros_and_trace("control_switch_vel", "sout")

        self.robot.add_trace("base_vel", "sout")
        self.robot.add_ros_and_trace("base_vel", "sout")

        self.robot.add_trace("biased_vel", "sout")
        self.robot.add_ros_and_trace("biased_vel", "sout")
