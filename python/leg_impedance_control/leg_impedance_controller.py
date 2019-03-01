## Contains implementation of leg impedance impedance_controller
## Author: Avadesh Meduri
## Date : 1st March 2019

#################### Imports #################################################

from leg_impedance_control.utils import *

from py_robot_properties_teststand.config import TeststandConfig

import dynamic_graph.sot.dynamics_pinocchio as dp


#############################################################################

class leg_impedance_controller():
    def __init__(self, leg_name):
        self.leg_name = leg_name
        self.robot_pin = TeststandConfig.buildRobotWrapper()
        self.robot_dg = dp.DynamicPinocchio(self.leg_name)
        self.robot_dg.setModel(self.robot_pin.model)
        self.robot_dg.setData(self.robot_pin.data)

        self.robot_dg.createJacobianEndEffWorld('jac_cnt' + self.leg_name, 'contact')
        self.robot_dg.createPosition('pos_hip' + self.leg_name, 'HFE')
        self.robot_dg.createPosition('pos_foot' + self.leg_name, 'END')

        print("Warning: Robot acceleration has been set to zero")
        self.robot_dg.acceleration.value = 3 * (0.0, )

    def compute_control_torques(self, start_index = 1, end_index = 3):
        '''
        ## computes torques tau = JacT*(errors)
        ## Input : start_index (to select actuated torques to plug ot the robot)
                 : end_index (to select actuated torques to plug to the robot)
        '''
        jacT = transpose_mat(self.jac, "jacTranspose" + self.leg_name)
        ## multiplying negative
        errors = mul_double_vec(-1.0, self.total_error, "neg_op" + self.leg_name)
        control_torques = multiply_mat_vec(jacT,errors, "compute_control_torques" + self.leg_name)

        ## selecting the torques to be plugged to the robot
        sel = Selec_of_vector("impedance_torques" + self.leg_name)
        sel.selec(start_index, end_index)
        plug(control_torques, sel.signal('sin'))
        return sel.signal('sout')

    def return_control_torques(self, kp, des_pos, kd = None, des_vel = None):
        '''
        ## Impedance controller implementation. Creates a virtual spring
        ## damper system the base and foot of the leg
        ## Input : Kp - proportional gain
                 : Kd - derivative gain
                 : des_pos - desired position
                 : des_vel - desired velocity
        '''

        self.jac = self.robot_dg.signal("jac_cnt" + self.leg_name)

        self.xyzpos_hip = hom2pos(self.robot_dg.signal("pos_hip" + self.leg_name), "xyzpos_hip" + self.leg_name)
        self.xyzpos_foot = hom2pos(self.robot_dg.signal("pos_foot" + self.leg_name), "xyzpos_foot" + self.leg_name)

        ## Relative position of foot with respect to the base of the foot
        self.rel_pos_foot = subtract_vec_vec(self.xyzpos_foot, self.xyzpos_hip, "rel_pos_foot" + self.leg_name)
        self.rel_pos_foot = stack_two_vectors(self.rel_pos_foot, constVector([0.0, 0.0, 0.0], 'stack_to_wrench' + self.leg_name), 3, 3)
        self.pos_error = subtract_vec_vec(self.rel_pos_foot, des_pos, "pos_error" + self.leg_name)
        mul_kp_gains = Multiply_double_vector("Kp" + self.leg_name)
        plug(kp, mul_kp_gains.sin1)
        plug(self.pos_error, mul_kp_gains.sin2)
        pos_error_with_gains = mul_kp_gains.sout

        if kd is not None and des_vel is not None:
            print("Kd !!!!!")
            self.rel_vel_foot = multiply_mat_vec(self.jac, self.robot_dg.velocity, "rel_vel_foot" + self.leg_name)
            self.vel_error = subtract_vec_vec(self.rel_vel_foot, des_vel, "vel_error" + self.leg_name)
            mul_kd_gains = Multiply_double_vector("Kd" + self.leg_name)
            plug(kd, mul_kd_gains.sin1)
            plug(self.vel_error, mul_kd_gains.sin2)
            vel_error_with_gains = mul_kd_gains.sout

            self.total_error = add_vec_vec(pos_error_with_gains, vel_error_with_gains, "total_error" + self.leg_name)

        else:
            self.total_error = pos_error_with_gains


        control_torques = self.compute_control_torques()

        return control_torques

# def get_leg_impedance_controller():
#     return leg_impedance_controller
