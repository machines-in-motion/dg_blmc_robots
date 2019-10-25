
# from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.utils import zero

from dg_blmc_robots.stuggihop import get_stuggihop_robot
# from dg_blmc_robots.stuggihop import StuggihopConfig
# import dynamic_graph.sot.dynamics_pinocchio as dyn_pin

from leg_impedance_control.utils import Add_of_double, constVector, stack_zero
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller

from dynamic_graph import plug

# Get the robot corresponding to the quadruped.
robot = get_stuggihop_robot()

# Define the desired position.
q = zero(robot.pin_robot.nq)
dq = zero(robot.pin_robot.nv)

q[0] = 0.4
q[1] = 0.25
q[2] = 0.8
q[3] = -1.6

# Update the initial state of the robot.
robot.reset_state(q, dq)

################################################################################

add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 100
kp = add_kp.sout

add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.01
kd = add_kd.sout

des_pos = constVector([0.0, 0.0, -0.2, 0.0, 0.0, 0.0],"pos_des")
des_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0],"vel_des")

################################################################################

leg_imp_ctrl = leg_impedance_controller("hopper")

plug(stack_zero(robot.device.signal('joint_positions'), 
    "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), 
    "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)


control_torques = leg_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel)

plug(control_torques, robot.device.ctrl_joint_torques)
################################################################################

robot.run(10000, 1./60.)