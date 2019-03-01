## impedance controller implementation on test stand
## Author : Avadesh Meduri
## Date : 1/03/19

########################### Imports ###########################################

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

###############################################################################


class quad_leg_impedance_controller():
    def __init__(self, robot):
        self.robot = robot

        self.imp_ctrl_leg_fl = leg_impedance_controller("leg_fl")
        self.imp_ctrl_leg_fr = leg_impedance_controller("leg_fr")
        self.imp_ctrl_leg_hl = leg_impedance_controller("leg_hl")
        self.imp_ctrl_leg_hr = leg_impedance_controller("leg_hr")

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
