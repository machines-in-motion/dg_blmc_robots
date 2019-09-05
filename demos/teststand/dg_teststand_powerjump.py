## power jump with teststand
## Author : Avadesh Meduri
## Date : 14/04/2019


from leg_impedance_control.utils import hom2pos, constVector
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from dynamic_graph_manager.dg_tools import power_jump_control


class PowerJump:
    def __init__(self, robot, name="power_jump"):
        self.name = name
        self.robot = robot

        self.des_weight_fff = constVector([0.0, 0.0, 1.8*9.81, 0.0, 0.0, 0.0], name + "_des_weight_fff")
        self.des_fff = constVector([0.0, 0.0, 4.0*9.81, 0.0, 0.0, 0.0], name + "_des_fff")
        self.des_pos_trigger = constVector([0.0, 0.0, -0.18, 0.0, 0.0, 0.0], name + "_pos_des_trigger")
        self.des_pos_air = constVector([0.0, 0.0, -0.25, 0.0, 0.0, 0.0], name + "_pos_des_air")
        self.des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], name + "_vel_des")

        self.des_kp_ground = constVector([100.0, 0.0, 250.0, 0.0, 0.0, 0.0], name + "_des_kp_ground")
        self.des_kp_air = constVector([50.0, 0.0, 150.0, 0.0, 0.0, 0.0], name + "_des_kp_air")
        self.des_kd = constVector([1.0, 0.0, 2.5, 0.0, 0.0, 0.0], name + "_des_kd_air")

        self.leg_imp_ctrl = leg_impedance_controller(name + "_leg_imp_ctrl")

        plug(stack_zero(robot.device.signal('joint_positions'), name + "_add_base_joint_position"),
                                                            self.leg_imp_ctrl.robot_dg.position)
        plug(stack_zero(robot.device.signal('joint_velocities'), name + "_add_base_joint_velocity"),
                                                            self.leg_imp_ctrl.robot_dg.velocity)

        self.leg_imp_ctrl.xyzpos_hip = hom2pos(self.leg_imp_ctrl.robot_dg.signal("pos_hip_" + self.leg_imp_ctrl.leg_name),
                                                                "xyzpos_hip_" + self.leg_imp_ctrl.leg_name)
        self.leg_imp_ctrl.xyzpos_foot = hom2pos(self.leg_imp_ctrl.robot_dg.signal("pos_foot_" + self.leg_imp_ctrl.leg_name),
                                                                "xyzpos_foot_" + self.leg_imp_ctrl.leg_name)

        self.leg_imp_ctrl.rel_pos_foot = subtract_vec_vec(self.leg_imp_ctrl.xyzpos_foot, self.leg_imp_ctrl.xyzpos_hip,
                                                                "rel_pos_foot_" + self.leg_imp_ctrl.leg_name)
        self.leg_imp_ctrl.rel_pos_foot = stack_two_vectors(self.leg_imp_ctrl.rel_pos_foot, constVector([0.0, 0.0, 0.0],
                                                        'stack_to_wrench_' + self.leg_imp_ctrl.leg_name), 3, 3)

        power_jump_ctrl = power_jump_control("power_jump_ctrl")
        plug(self.leg_imp_ctrl.rel_pos_foot, power_jump_ctrl.leg_length) #6d vector
        plug(robot.device.contact_sensors, power_jump_ctrl.cnt_sensor)
        plug(des_pos_trigger, power_jump_ctrl.leg_length_trigger)
        plug(des_pos_air, power_jump_ctrl.leg_length_air)
        plug(des_fff, power_jump_ctrl.des_fff)
        plug(des_weight_fff, power_jump_ctrl.des_weight_fff)
        plug(des_kp_ground, power_jump_ctrl.kp_ground)
        plug(des_kp_air, power_jump_ctrl.kp_air)


        des_pos = power_jump_ctrl.des_pos
        des_force = power_jump_ctrl.des_force
        des_kp = power_jump_ctrl.des_kp

        ##For making gain input dynamic through terminal
        add_kf = Add_of_double('kf')
        add_kf.sin1.value = 0
        ### Change this value for different gains
        add_kf.sin2.value = 1.0
        kf = add_kf.sout


        control_torques = self.leg_imp_ctrl.return_control_torques(des_kp, des_pos,
                                                            des_kd, des_vel, kf, des_force)


        plug(control_torques, robot.device.ctrl_joint_torques)
        ###############################################################################
        #
        leg_imp_ctrl.record_data(robot)

        robot.add_trace("power_jump_ctrl", "des_pos")
        robot.add_ros_and_trace("power_jump_ctrl", "des_pos")

        robot.add_trace("power_jump_ctrl", "des_force")
        robot.add_ros_and_trace("power_jump_ctrl", "des_force")

        robot.add_trace("power_jump_ctrl", "des_kp")
        robot.add_ros_and_trace("power_jump_ctrl", "des_kp")

if ('robot' in globals()) or ('robot' in locals()):
    power_jump = PowerJump(robot)