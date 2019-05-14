## impedance controller implementation on test stand
## Author : Avadesh Meduri
## Date : 1/03/19

########################### Imports ###########################################

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2
# from dynamic_graph_manager.vicon_sdk import ViconClientEntity
from dynamic_graph_manager.dg_tools import ComImpedanceControl
from dynamic_graph.sot.core.switch import SwitchVector

import dynamic_graph.sot.dynamics_pinocchio as dp

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
    def __init__(self, robot, ViconClientEntity, client_name = "vicon_client" , vicon_ip = '10.32.3.16:801', EntityName = "quad_com_ctrl"):

        self.robot = robot

        self.EntityName = EntityName
        self.client_name = client_name
        self.host_name_quadruped = vicon_ip
        self.robot_vicon_name = "quadruped"

        self.vicon_client = ViconClientEntity(self.client_name)
        self.vicon_client.connect_to_vicon(self.host_name_quadruped)
        self.vicon_client.displaySignals()

        self.vicon_client.add_object_to_track("{}/{}".format(self.robot_vicon_name, self.robot_vicon_name))

        try:
            ## comment out if running on real robot
            self.vicon_client.robot_wrapper(robot)
        except:
            print("not in simulation")


        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_position")
        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_velocity_body")
        self.robot.add_ros_and_trace(self.client_name, self.robot_vicon_name + "_velocity_world")

        self.com_imp_ctrl = ComImpedanceControl(EntityName)

    def compute_torques(self, Kp, des_pos, Kd, des_vel, des_fff):


        self.base_pos_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_position"),
                                        0, 3, "base_pos")
        self.base_vel_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"),
                                        0, 3, "base_vel")


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
        self.com_imp_ctrl.mass.value = [2.17784, 2.17784, 2.17784]

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

    def compute_ang_control_torques(self, Kp_ang, des_ori, Kd_ang, des_ang_vel, des_fft):

        """
        ### Computes torques required to control the orientation of base
        """

        self.base_orientation = selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                             + "_position"), 3,7, "base_orientation")
        self.base_ang_vel_xyz =  selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                        + "_velocity_body"),3, 6, "selec_ang_dxyz")

        plug(Kp_ang, self.com_imp_ctrl.Kp_ang)
        plug(Kd_ang, self.com_imp_ctrl.Kd_ang)
        self.com_imp_ctrl.inertia.value = [0.00578574, 0.01938108, 0.02476124]
        plug(self.base_orientation, self.com_imp_ctrl.ori)
        plug(self.base_ang_vel_xyz, self.com_imp_ctrl.angvel)
        plug(des_ori, self.com_imp_ctrl.des_ori)
        plug(des_ang_vel, self.com_imp_ctrl.des_ang_vel)
        plug(des_fft, self.com_imp_ctrl.des_fft)

        return self.com_imp_ctrl.angtau


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

    def return_lqr_tau(self,des_pos, des_vel, des_ori, des_ang_vel, des_fff, des_fft, des_lqr):
        """
        ### for lqr based controller. reads values obtained from the planner
        """

        self.base_pos_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_position"),
                                        0, 3, "base_pos")
        self.base_vel_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"),
                                        0, 3, "base_vel")


        plug(self.base_pos_xyz, self.com_imp_ctrl.position)
        plug(self.base_vel_xyz, self.com_imp_ctrl.velocity)

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

        self.base_orientation = selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                             + "_position"), 3,7, "base_orientation")
        self.biased_base_pos_xyz = subtract_vec_vec(self.base_pos_xyz,
                                        self.control_switch_pos.sout, "biased_pos")
        self.biased_base_vel_xyz = subtract_vec_vec(self.base_vel_xyz,
                                        self.control_switch_vel.sout, "biased_vel")
        self.base_ang_vel_xyz =  selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                        + "_velocity_body"),3, 6, "selec_ang_dxyz")

        #### LQR equation:
        ###should be removed
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kp)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kd)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kp_ang)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kd_ang)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.inertia)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.mass)

        ###
        plug(des_lqr, self.com_imp_ctrl.lqr_gain)
        plug(self.biased_base_pos_xyz, self.com_imp_ctrl.biased_pos)
        plug(des_pos, self.com_imp_ctrl.des_pos)
        plug(self.biased_base_vel_xyz, self.com_imp_ctrl.biased_vel)
        plug(des_vel, self.com_imp_ctrl.des_vel)
        plug(des_fff, self.com_imp_ctrl.des_fff)
        plug(self.base_orientation, self.com_imp_ctrl.ori)
        plug(self.base_ang_vel_xyz, self.com_imp_ctrl.angvel)
        plug(des_ori, self.com_imp_ctrl.des_ori)
        plug(des_ang_vel, self.com_imp_ctrl.des_ang_vel)
        plug(des_fft, self.com_imp_ctrl.des_fft)

        lqr_com_force = self.com_imp_ctrl.lqrtau

        return lqr_com_force

    def return_end_eff_lqr_tau(self, des_pos, des_vel, des_ori, des_ang_vel, des_fff, des_lqr ):
        """
        ### for lqr based controller at the end effector. reads values obtained from the planner
        """

        self.base_pos_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_position"),
                                        0, 3, "base_pos")
        self.base_vel_xyz = selec_vector(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"),
                                        0, 3, "base_vel")


        plug(self.base_pos_xyz, self.com_imp_ctrl.position)
        plug(self.base_vel_xyz, self.com_imp_ctrl.velocity)

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

        self.base_orientation = selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                             + "_position"), 3,7, "base_orientation")
        self.biased_base_pos_xyz = subtract_vec_vec(self.base_pos_xyz,
                                        self.control_switch_pos.sout, "biased_pos")
        self.biased_base_vel_xyz = subtract_vec_vec(self.base_vel_xyz,
                                        self.control_switch_vel.sout, "biased_vel")
        self.base_ang_vel_xyz =  selec_vector(self.vicon_client.signal(self.robot_vicon_name
                                        + "_velocity_body"),3, 6, "selec_ang_dxyz")

        #### LQR equation:
        ###should be removed
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kp)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kd)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kp_ang)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.Kd_ang)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.inertia)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.mass)
        plug(constVector([0.0, 0.0, 0.0], "Kp_ang"), self.com_imp_ctrl.des_fft)


        ###
        plug(des_lqr, self.com_imp_ctrl.lqr_gain)
        plug(self.biased_base_pos_xyz, self.com_imp_ctrl.biased_pos)
        plug(des_pos, self.com_imp_ctrl.des_pos)
        plug(self.biased_base_vel_xyz, self.com_imp_ctrl.biased_vel)
        plug(des_vel, self.com_imp_ctrl.des_vel)
        plug(des_fff, self.com_imp_ctrl.des_fff)
        plug(self.base_orientation, self.com_imp_ctrl.ori)
        plug(self.base_ang_vel_xyz, self.com_imp_ctrl.angvel)
        plug(des_ori, self.com_imp_ctrl.des_ori)
        plug(des_ang_vel, self.com_imp_ctrl.des_ang_vel)

        lqr_end_eff_force = self.com_imp_ctrl.end_eff_lqr_tau

        torques_fl = selec_vector(lqr_end_eff_force, 0, 3, self.EntityName + "torques_fl")

        torques_fr = selec_vector(lqr_end_eff_force, 3, 6, self.EntityName + "torques_fr")

        torques_hl = selec_vector(lqr_end_eff_force, 6, 9, self.EntityName + "torques_hl")

        torques_hr = selec_vector(lqr_end_eff_force, 9, 12, self.EntityName + "torques_hr")

        torques_fl_6d = stack_two_vectors(torques_fl,
                                            zero_vec(3, "stack_fl_tau"), 3, 3)
        torques_fr_6d = stack_two_vectors(torques_fr,
                                            zero_vec(3, "stack_fr_tau"), 3, 3)
        torques_hl_6d = stack_two_vectors(torques_hl,
                                            zero_vec(3, "stack_hl_tau"), 3, 3)
        torques_hr_6d = stack_two_vectors(torques_hr,
                                            zero_vec(3, "stack_hr_tau"), 3, 3)

        torques_fl_fr = stack_two_vectors(torques_fl_6d,torques_fr_6d, 6, 6)
        torques_hl_hr = stack_two_vectors(torques_hl_6d,torques_hr_6d, 6, 6)

        lqr_end_eff_force_24d = stack_two_vectors(torques_fl_fr, torques_hl_hr, 12, 12)
        lqr_end_eff_force_24d = mul_double_vec(1.0, lqr_end_eff_force_24d,"lqr_end_eff_force")


        return lqr_end_eff_force_24d


    def return_com_torques(self, com_tau, ang_tau, des_abs_vel, hess, g0, ce, ci, ci0, reg, cnt_plan = None):
        ### This divides forces using the wbc controller

        plug(com_tau, self.com_imp_ctrl.lctrl)
        plug(ang_tau, self.com_imp_ctrl.actrl)
        plug(des_abs_vel, self.com_imp_ctrl.abs_end_eff_vel)
        plug(hess, self.com_imp_ctrl.hess)
        plug(g0, self.com_imp_ctrl.g0)
        plug(ce, self.com_imp_ctrl.ce)
        plug(ci, self.com_imp_ctrl.ci)
        plug(ci0, self.com_imp_ctrl.ci0)
        plug(reg, self.com_imp_ctrl.reg)
        if cnt_plan == None:
            thr_cnt_sensor = self.threshold_cnt_sensor()
            plug(thr_cnt_sensor, self.com_imp_ctrl.thr_cnt_value)

        else:
            plug(cnt_plan, self.com_imp_ctrl.cnt_sensor)
            plug(cnt_plan, self.com_imp_ctrl.thr_cnt_value)


        self.wb_ctrl = self.com_imp_ctrl.wbctrl

        ## Thresholding with contact sensor to make forces event based
        # fl_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 0, 1, "fl_cnt_3d")
        # fr_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 1, 2, "fr_cnt_3d")
        # hl_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 2, 3, "hl_cnt_3d")
        # hr_cnt_value = self.convert_cnt_value_to_3d(thr_cnt_sensor, 3, 4, "hr_cnt_3d")

        torques_fl = selec_vector(self.wb_ctrl, 0, 3, self.EntityName + "torques_fl")
#        torques_fl = mul_vec_vec(fl_cnt_value, torques_fl, self.EntityName + "fused_torques_fl")

        torques_fr = selec_vector(self.wb_ctrl, 3, 6, self.EntityName + "torques_fr")
#        torques_fr = mul_vec_vec(fr_cnt_value, torques_fr, self.EntityName + "fused_torques_fr")

        torques_hl = selec_vector(self.wb_ctrl, 6, 9, self.EntityName + "torques_hl")
#        torques_hl = mul_vec_vec(hl_cnt_value, torques_hl, self.EntityName + "fused_torques_hl")

        torques_hr = selec_vector(self.wb_ctrl, 9, 12, self.EntityName + "torques_hr")
#        torques_hr = mul_vec_vec(hr_cnt_value, torques_hr, self.EntityName + "fused_torques_hr")

        torques_fl_6d = stack_two_vectors(torques_fl,
                                            zero_vec(3, "stack_fl_tau"), 3, 3)
        torques_fr_6d = stack_two_vectors(torques_fr,
                                            zero_vec(3, "stack_fr_tau"), 3, 3)
        torques_hl_6d = stack_two_vectors(torques_hl,
                                            zero_vec(3, "stack_hl_tau"), 3, 3)
        torques_hr_6d = stack_two_vectors(torques_hr,
                                            zero_vec(3, "stack_hr_tau"), 3, 3)

        torques_fl_fr = stack_two_vectors(torques_fl_6d,torques_fr_6d, 6, 6)
        torques_hl_hr = stack_two_vectors(torques_hl_6d,torques_hr_6d, 6, 6)

        wbc_torques = stack_two_vectors(torques_fl_fr, torques_hl_hr, 12, 12)
        ## hack to allow tracking of torques
        wbc_torques = mul_double_vec(1.0, wbc_torques,"com_torques")

        return wbc_torques



    def record_data(self):
        # self.robot.add_trace(self.EntityName, "tau")
        # self.robot.add_ros_and_trace(self.EntityName, "tau")
        # #
        # self.robot.add_trace(self.EntityName, "angtau")
        # self.robot.add_ros_and_trace(self.EntityName, "angtau")
        # #
        #
        self.robot.add_trace(self.EntityName, "wbctrl")
        self.robot.add_ros_and_trace(self.EntityName, "wbctrl")

        self.robot.add_trace(self.EntityName, "lqrtau")
        self.robot.add_ros_and_trace(self.EntityName, "lqrtau")

        # self.robot.add_trace(self.EntityName, "thr_cnt_sensor")
        # self.robot.add_ros_and_trace(self.EntityName, "thr_cnt_sensor")
        #
        self.robot.add_trace("com_torques", "sout")
        self.robot.add_ros_and_trace("com_torques", "sout")

        # self.robot.add_trace(self.EntityName, "end_eff_lqr_tau")
        # self.robot.add_ros_and_trace(self.EntityName, "end_eff_lqr_tau")

        # self.robot.add_trace("lqr_end_eff_force", "sout")
        # self.robot.add_ros_and_trace("lqr_end_eff_force", "sout")

        self.robot.add_trace("biased_pos", "sout")
        self.robot.add_ros_and_trace("biased_pos", "sout")

        #
        # self.robot.add_trace("biased_vel", "sout")
        # self.robot.add_ros_and_trace("biased_vel", "sout")
        # #
        # self.robot.add_trace("quad_com_ctrl", "lqrtau")
        # self.robot.add_ros_and_trace("quad_com_ctrl", "lqrtau")
        #
        # self.robot.add_trace("lqr_error", "sout")
        # self.robot.add_ros_and_trace("lqr_error", "sout")
        #
        # self.robot.add_trace("lqr_com_force", "sout")
        # self.robot.add_ros_and_trace("lqr_com_force", "sout")
        #
        # self.robot.add_trace("f_hr_3d", "sout")
        # self.robot.add_ros_and_trace("f_hr_3d", "sout")
        #
        #
        # self.robot.add_trace("des_fff_lqr", "sout")
        # self.robot.add_ros_and_trace("des_fff_lqr", "sout")
