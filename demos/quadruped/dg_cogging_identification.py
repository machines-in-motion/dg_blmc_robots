from dynamic_graph.sot.torque_control.position_controller import PositionController
from dynamic_graph.sot.torque_control.control_manager import ControlManager
from dynamic_graph.sot.torque_control.joint_trajectory_generator import JointTrajectoryGenerator
from dynamic_graph.sot.core import Stack_of_vector
from dynamic_graph import plug
from leg_impedance_control import control_manager_conf
from leg_impedance_control import joint_pos_ctrl_gains
from leg_impedance_control.utils import stack_two_vectors

NJ = control_manager_conf.nbJoints
dt = 0.001

def create_ctrl_manager(conf, motor_params, dt, robot_name='robot'):
    ctrl_manager = ControlManager("ctrl_man");

    ctrl_manager.tau_predicted.value    = NJ*(0.0,);
    ctrl_manager.i_measured.value       = NJ*(0.0,);
    ctrl_manager.tau_max.value          = NJ*(conf.TAU_MAX,);
    ctrl_manager.i_max.value            = NJ*(conf.CURRENT_MAX,);
    ctrl_manager.u_max.value            = NJ*(conf.CTRL_MAX,);

    # Init should be called before addCtrlMode
    # because the size of state vector must be known.
    ctrl_manager.init(dt, conf.urdfFileName, robot_name)

    # Set the map from joint name to joint ID
    for key in conf.mapJointNameToID:
      ctrl_manager.setNameToId(key,conf.mapJointNameToID[key])

    # Set the map joint limits for each id
    for key in conf.mapJointLimits:
      ctrl_manager.setJointLimitsFromId(key,conf.mapJointLimits[key][0], \
                              conf.mapJointLimits[key][1])

    # Set the force limits for each id
    for key in conf.mapForceIdToForceLimits:
      ctrl_manager.setForceLimitsFromId(key,tuple(conf.mapForceIdToForceLimits[key][0]), \
                              tuple(conf.mapForceIdToForceLimits[key][1]))

    # Set the force sensor id for each sensor name
    for key in conf.mapNameToForceId:
      ctrl_manager.setForceNameToForceId(key,conf.mapNameToForceId[key])

    # Set the map from the urdf joint list to the sot joint list
    ctrl_manager.setJointsUrdfToSot(conf.urdftosot)

    # Set the foot frame name
    # for key in conf.footFrameNames:
    #   ctrl_manager.setFootFrameName(key,conf.footFrameNames[key])

    # Set IMU hosting joint name
    # ctrl_manager.setImuJointName(conf.ImuJointName)
    # ctrl_manager.setRightFootForceSensorXYZ(conf.rightFootSensorXYZ);
    # ctrl_manager.setRightFootSoleXYZ(conf.rightFootSoleXYZ);

    return ctrl_manager;

def create_base6d_encoders(robot):
    robot.base6d_encoders = Stack_of_vector("")
    robot.base6d_encoders.selec1(0, 6)
    robot.base6d_encoders.selec2(0, NJ)
    robot.base6d_encoders.sin1.value = 6*(0.0,)
    plug(robot.device.joint_positions, robot.base6d_encoders.sin2)

def create_position_controller(robot, gains, dt, robot_name="robot"):
    posCtrl = PositionController('pos_ctrl')
    posCtrl.Kp.value = tuple(gains.kp_pos[round(dt,3)])
    posCtrl.Kd.value = tuple(gains.kd_pos[round(dt,3)])
    posCtrl.Ki.value = tuple(gains.ki_pos[round(dt,3)])
    posCtrl.dqRef.value = NJ*(0.0,);

    plug(robot.base6d_encoders.sout,       posCtrl.base6d_encoders)
    plug(robot.device.joint_velocities,    posCtrl.jointsVelocities)
    plug(posCtrl.pwmDes,                   robot.device.ctrl_joint_torques)
    plug(robot.traj_gen.q,       posCtrl.qRef)
    posCtrl.init(dt, robot_name)
    return posCtrl

def create_trajectory_generator(robot, dt=0.001, robot_name="robot"):
    jtg = JointTrajectoryGenerator("jtg");
    plug(robot.base6d_encoders.sout,             jtg.base6d_encoders);
    jtg.init(dt, robot_name);
    return jtg;


robot.ctrl_manager = create_ctrl_manager(control_manager_conf, None, dt)
create_base6d_encoders(robot)
robot.traj_gen = create_trajectory_generator(robot, dt)
robot.pos_ctrl = create_position_controller(robot, joint_pos_ctrl_gains, dt)
