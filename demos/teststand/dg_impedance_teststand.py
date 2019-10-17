## simple impedance controller implementation on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

class impedance:
    def __init__(self, robot, name="impedance"):
        self.name = name
        self.robot = robot
        self.kp_split = constVector([30.0, 0.0, 30.0, 0.0, 0.0, 0.0], "kp_split")

        self.kd_split = constVector([0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "kd_split")

        self.des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
        self.des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")


        self.leg_imp_ctrl = leg_impedance_controller("hopper")

        plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), self.leg_imp_ctrl.robot_dg.position)
        plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), self.leg_imp_ctrl.robot_dg.velocity)

        self.control_torques = self.leg_imp_ctrl.return_control_torques(self.kp_split, self.des_pos, self.kd_split, self.des_vel)

        plug(self.control_torques, robot.device.ctrl_joint_torques)
        robot.add_to_ros("jacTranspose" + "hopper", "sout", topic_type= 'matrix')
        robot.add_to_ros("add_base_joint_velocity", "sout")


        robot.add_to_ros("impedance_torques_" + "hopper", "sout")
        robot.add_to_ros("neg_op_" + "hopper", "sout")
        robot.add_to_ros("sel_torques_" + "hopper", "sout")
        robot.add_to_ros("friction", "sout")

        ###################### Record Data ##########################################################
        # leg_imp_ctrl.record_data(robot)

if ('robot' in globals()) or ('robot' in locals()):
    power_jump = impedance(robot)