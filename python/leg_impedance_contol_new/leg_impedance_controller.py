## Contains implementation of leg impedance impedance_controller
## Author: Avadesh Meduri
## Date : 1st March 2019

#################### Imports #################################################

from leg_impedance_control.utils import *


import pinocchio as se3
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero

from py_robot_properties_teststand.config import TeststandConfig

import dynamic_graph.sot.dynamics_pinocchio as dp


#############################################################################

class leg_impedance_controller(leg_name):
    def __init__(self, leg_name):
        self.leg_name = name
        self.robot_pin = TeststandConfig.buildRobotWrapper()
        self.robot_dg = dp.DynamicPinocchio(self.leg_name)
        self.robot_dg.setModel(self.robot_pin.model)
        self.robot_dg.setData(self.robot_pin.data)

        self.robot_dg.createJacobianEndEffWorld('jac_cnt' + self.leg_name, 'contact')
        self.robot_dg.createPosition('pos_hip' + self.leg_name, 'HFE')
        self.robot_dg.createPosition('pos_foot' + self.leg_name, 'END')

    def compute_control_torques(self, start_index = 1, end_index = 3):
        '''
        ## computes torques tau = JacT*(errors)
        ## Input : start_index (to select actuated torques to plug ot the robot)
                 : end_index (to select actuated torques to plug to the robot)
        '''
        jacT = transpose_mat(self.jac, "jacTranspose" + self.leg_name)
        ## multiplying negative
        errors = mul_double_vec(-1.0, errors, "neg_op" + self.leg_name)
        control_torques = mul_mat_vec(jacT,errors, "compute_control_torques" + self.leg_name)

        ## selecting the torques to be plugged to the robot
        sel = Selec_of_vector("impedance_torques" + self.leg_name)
        sel.selec(start_index, end_index)
        plug(control_torques, sel.signal('sin'))
        return sel.signal('sout')

    def impedance_controller(self, kp, kd = None, des_pos, des_vel = None):
        '''
        ## Impedance controller implementation. Creates a virtual spring
        ## damper system the base and foot of the leg
        ## Input : Kp - proportional gain
                 : Kd - derivative gain
                 : des_pos - desired position
                 : des_vel - desired velocity
        '''

        jac = self.robot_dg.signal("jac_cnt" + self.leg_name)

        self.xyzpos_hip = hom2pos(self.robot_dg.signal("pos_hip" + self.leg_name), "xyzpos_hip" + self.leg_name)
        self.xyzpos_foot = hom2pos(self.robot_dg.signal("pos_foot" + self.leg_name), "xyzpos_foot" + self.leg_name)

        ## Relative position of foot with respect to the base of the foot
        self.rel_pos_foot = subtract_vec_vec(self.xyzpos_foot, self.xyzpos_hip, "rel_pos_foot" + self.leg_name)
        pos_error = subtract_vec_vec(rel_pos_foot, des_pos, "pos_error" + self.leg_name)
        self.pos_error = stack_two_vectors(pos_error, constVector([0.0, 0.0, 0.0], 'stack_to_wrench' + self.leg_name), 3, 3)

        mul_Kp_gains = Multiply_double_vector("Kp" + self.leg_name)
        plug(Kp, mul_kp_gains.sin1)
        plug(pos_error, mul_kp_gains.sin2)
        pos_error_with_gains = mul_Kp_gains.sout

        if Kd is not None and des_vel is not None:
            self.rel_vel_foot = multiply_mat_vec(jac, self.robot_dg.velocity, "rel_vel_foot" + self.leg_name)
            self.vel_error = subtract_vec_vec(self.rel_vel_foot, des_vel, "vel_error" + self.leg_name)
            mul_Kd_gains = Multiply_double_vector("Kd" + self.leg_name)
            plug(Kd, mul_Kd_gains.sin1)
            plug(self.vel_error, mul_Kd_gains.sin2)
            vel_error_with_gains = mul_Kd_gains.sout

            total_error = add_vec_vec(pos_error_with_gains + vel_error_with_gains, "total_error" + self.leg_name)

        else:
            total_error = pos_error_with_gains


        control_torques = compute_control_torques(jac, total_error)

        return control_torques

# def get_leg_impedance_controller():
#     return leg_impedance_controller
