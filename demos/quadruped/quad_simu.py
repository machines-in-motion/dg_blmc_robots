## For debugging


from leg_impedance_control.utils import *
from leg_impedance_control.controller import *
from leg_impedance_control.traj_generators import *

from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot


###### robot init #######################################################

local_device = Device("hopper_robot")
yaml_path = "./../../../../robots/robot_properties/robot_properties_teststand/config/teststand.yaml"
local_device.initialize(yaml_path)
robot = Robot(name=local_device.name, device=local_device)



######## robot simulation ################################################

from pinocchio.utils import zero

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q = zero(2)
dq = zero(2)
q_out = q.T.tolist()[0]
dq_out = dq.T.tolist()[0]
ddq = zero(2)
joint_torques = zero(2)

q[:] = np.asmatrix(robot.device.joint_positions.value).T
dq[:] = np.asmatrix(robot.device.joint_velocities.value).T

print(q, dq)

dt = 0.001#config.dt
motor_inertia = 0.045

for i in range(40000):
    # fill the sensors
    robot.device.joint_positions.value = q.T.tolist()[0][:]
    robot.device.joint_velocities.value = dq.T.tolist()[0][:]

    # "Execute the dynamic graph"
    robot.device.execute_graph()


    joint_torques[:] = np.asmatrix(robot.device.ctrl_joint_torques.value).T
    #joint_torques[:] = control_torques.value

    #print(q, dq)
    pos_des_fl_fr.recompute(i)
    print(pos_des_fl_fr.value)

    # integrate the configuration from the computed torques
    #q = (q + dt * dq + dt * dt * 0.5 * joint_torques / motor_inertia)
    #dq = (dq + dt * joint_torques / motor_inertia)
    q = (q + dt * dq + dt * dt * 0.5 * 0.01 / motor_inertia)
    dq = (dq + dt * 0.01 / motor_inertia)


    if (i % 1000) == 0:
        # print "qref =     ", robot.pid_control.pose_controller.qRef.value
        # print ("q =        ",
        #        robot.pid_control.pose_controller.base6d_encoders.value[6:])
        # print "err_pid =  ", robot.pid_control.pose_controller.qError.value
        # print "currents = ", robot.device.ctrl_joint_torques.value
        pass


print
print "End of simulation"
print "control_torques = ", robot.device.ctrl_joint_torques.value
