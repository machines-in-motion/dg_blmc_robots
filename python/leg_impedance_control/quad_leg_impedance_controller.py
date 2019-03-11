## impedance controller implementation on test stand
## Author : Avadesh Meduri
## Date : 1/03/19

########################### Imports ###########################################

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from dynamic_graph_manager.vicon_sdk import ViconClientEntity


###############################################################################


class quad_leg_impedance_controller():
    def __init__(self, robot):
        self.robot = robot

        self.imp_ctrl_leg_fl = leg_impedance_controller("_fl")
        self.imp_ctrl_leg_fr = leg_impedance_controller("_fr")
        self.imp_ctrl_leg_hl = leg_impedance_controller("_hl")
        self.imp_ctrl_leg_hr = leg_impedance_controller("_hr")

    def return_control_torques(self, kp, des_pos, kd=None, des_vel=None):
        """
        Input : Kp - proportional gain (double)
              : des_pos - 1*12 vector of desired_position in current time step
              : Kd - derivative gain (double)
              : des_vel - 1*12 vector of desired_velocity in current time step
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

        control_torques_fl = self.imp_ctrl_leg_fl.return_control_torques(kp, des_pos_fl, kd, des_vel_fl)

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

        control_torques_fr = self.imp_ctrl_leg_fr.return_control_torques(kp, des_pos_fr, kd, des_vel_fr)

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

        control_torques_hl = self.imp_ctrl_leg_hl.return_control_torques(kp, des_pos_hl, kd, des_vel_hl)

        ## For HR ##############################################################

        joint_positions_hr = selec_vector(self.joint_positions, 6, 8, 'position_slicer_hr')
        joint_velocities_hr = selec_vector(self.joint_velocities, 6, 8, 'velocity_slicer_hr')

        plug(stack_zero((joint_positions_hr),"add_base_joint_position_hr"), self.imp_ctrl_leg_hr.robot_dg.position)
        plug(stack_zero((joint_velocities_hr), "add_base_joint_velocity_hr"), self.imp_ctrl_leg_hr.robot_dg.velocity )

        des_pos_hr = selec_vector(des_pos, 12, 18, 'des_position_slicer_hr')
        if des_vel is not None:
            des_vel_hr = selec_vector(des_vel, 12, 18, 'des_veolcity_slicer_hr')
        else:
            des_vel_hr = None

        control_torques_hr = self.imp_ctrl_leg_hr.return_control_torques(kp, des_pos_hr, kd, des_vel_hr)

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
    def __init__(self, robot, client_name = "vicon_client" , vicon_ip = '10.32.24.190:801'):

        self.robot = robot
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


        self.vicon_client.signal(self.robot_vicon_name + "_position").recompute(0)
        self.com_pos_bias = self.vicon_client.signal(self.robot_vicon_name + "_position")
        self.vicon_client.signal(self.robot_vicon_name + "_velocity_body").recompute(0)
        self.com_vel_bias = self.vicon_client.signal(self.robot_vicon_name + "_velocity_body")


        ### To bring all COM values to a relatice co-ordinate frame
        for t in range(1, 1000):
            self.vicon_client.signal(self.robot_vicon_name + "_position").recompute(t)
            self.com_pos_bias = add_vec_vec(self.vicon_client.signal(self.robot_vicon_name + "_position"),
                                            self.com_pos_bias, "pos_bias_" + str(t))

            self.vicon_client.signal(self.robot_vicon_name + "_velocity_body").recompute(t)
            self.com_vel_bias = add_vec_vec(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"),
                                            self.com_vel_bias, "vel_bias_" + str(t))

        self.com_pos_bias = mul_double_vec(0.001, self.com_pos_bias, "com_pos_bias")
        self.com_vel_bias = mul_double_vec(0.001, self.com_vel_bias, "com_vel_bias")

    def return_com_control_torques(self, kp, des_pos, kd = None, des_vel = None):

        self.com_pos = subtract_vec_vec(self.vicon_client.signal(self.robot_vicon_name + "_position"), self.com_pos_bias, "com_position")
        self.com_vel = subtract_vec_vec(self.vicon_client.signal(self.robot_vicon_name + "_velocity_body"), self.com_vel_bias, "com_velocity")

        self.pos_error = subtract_vec_vec(self.com_pos, des_pos, "com_pos_error")
        mul_kp_gains = Multiply_double_vector("Kp_com")
        plug(kp, mul_kp_gains.sin1)
        plug(self.pos_error, mul_kp_gains.sin2)
        self.pos_error_with_gains = mul_kp_gains.sout

        if des_vel is not None:
            self.vel_error = subtract_vec_vec(self.com_vel, des_vel, "com_vel_error")
            print("Kd !!!!!")
            mul_kd_gains = Multiply_double_vector("Kd_com")
            plug(kd, mul_kd_gains.sin1)
            plug(self.vel_error, mul_kd_gains.sin2)
            self.vel_error_with_gains = mul_kd_gains.sout

            self.total_error = add_vec_vec(self.pos_error_with_gains, self.vel_error_with_gains, "total_error")

        else:
            self.total_error = self.pos_error_with_gains

        return self.total_error

    def record_data(self):

        self.robot.add_trace("com_position", "sout")
        self.robot.add_ros_and_trace("com_position", "sout")

        self.robot.add_trace("com_velocity", "sout")
        self.robot.add_ros_and_trace("com_velocity", "sout")
