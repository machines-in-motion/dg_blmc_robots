# Impedance controller for quadruped taking independent jacobians for each leg for standing
# Author : Avadesh Meduri
# Date : 29 Jan 2019



from utils import *

#############################################################################



def quad_impedance_controller(robot, pos_des_fl, pos_des_fr, pos_des_hl, pos_des_hr, Kp):

    robot_fl_py, robot_fl_dg, robot_fr_py, robot_fr_dg, robot_hl_py, robot_hl_dg, robot_hr_py, robot_hr_dg = createLegs()

    #############################################################################
    # from dynamic_graph.sot.core.control_pd import ControlPD

    # pd = ControlPD("PDController")
    # pd.Kp.value = [0., 0.]
    # pd.Kd.value = [0.1,0.1]
    # pd.desiredposition.value = (0., 0.)
    # pd.desiredvelocity.value = (0., 0.)
    #robot_dg.acceleration.value = 3 * (0.0, )

    ##### torques for fl

    joint_positions = robot.device.signal('joint_positions')
    joint_velocities = robot.device.signal('joint_velocities')


    joint_positions_fl = selec_vector(joint_positions, 0, 2, 'position_slicer_fl')
    joint_velocities_fl = selec_vector(joint_velocities, 0, 2, 'velocity_slicer_fl')

    plug(stack_zero((joint_positions_fl),"add_base_joint_position_fl"), robot_fl_dg.position)
    plug(stack_zero((joint_velocities_fl), "add_base_joint_velocity_fl"), robot_fl_dg.velocity)
    robot_fl_dg.acceleration.value = 3 * (0.0, )

    control_torques_fl = impedance_controller_fl(robot_fl_dg,Kp,pos_des_fl)

    ##### torques for fr

    joint_positions_fr = selec_vector(joint_positions, 2, 4, 'position_slicer_fr')
    joint_velocities_fr = selec_vector(joint_velocities, 2, 4, 'velocity_slicer_fr')

    plug(stack_zero((joint_positions_fr),"add_base_joint_position_fr"), robot_fr_dg.position)
    plug(stack_zero((joint_velocities_fr), "add_base_joint_velocity_fr"), robot_fr_dg.velocity)
    robot_fr_dg.acceleration.value = 3 * (0.0, )

    control_torques_fr = impedance_controller_fr(robot_fr_dg, Kp,pos_des_fr)

    ##### torques for hl

    joint_positions_hl = selec_vector(joint_positions, 4, 6, 'position_slicer_hl')
    joint_velocities_hl = selec_vector(joint_velocities, 4, 6, 'velocity_slicer_hl')

    plug(stack_zero((joint_positions_hl),"add_base_joint_position_hl"), robot_hl_dg.position)
    plug(stack_zero((joint_velocities_hl), "add_base_joint_velocity_hl"), robot_hl_dg.velocity)
    robot_hl_dg.acceleration.value = 3 * (0.0, )

    control_torques_hl = impedance_controller_hl(robot_hl_dg,Kp,pos_des_hl)

    ###### torques for hr
    joint_positions_hr = selec_vector(joint_positions, 6, 8, 'position_slicer_hr')
    joint_velocities_hr = selec_vector(joint_velocities, 6, 8, 'velocity_slicer_hr')

    plug(stack_zero((joint_positions_hr),"add_base_joint_position_hr"), robot_hr_dg.position)
    plug(stack_zero((joint_velocities_hr), "add_base_joint_velocity_hr"), robot_hr_dg.velocity)
    robot_hr_dg.acceleration.value = 3 * (0.0, )

    control_torques_hr = impedance_controller_hr(robot_hr_dg,Kp,pos_des_hr)


    ############################################

    control_torques_fl_fr = stack_two_vectors(control_torques_fl, control_torques_fr, 2, 2)
    control_torques_hl_hr = stack_two_vectors(control_torques_hl, control_torques_hr, 2, 2)

    control_torques = stack_two_vectors(control_torques_fl_fr,control_torques_hl_hr, 4, 4)

    return control_torques
