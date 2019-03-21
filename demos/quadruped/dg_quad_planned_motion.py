## This code attempts to read a trajectory from a file and track it with the compliance CONTROLLER

## Author: Avadesh Meduri
## Date: 21/02/2019
from leg_impedance_control.utils import *
from leg_impedance_control.quad_leg_impedance_controller import quad_leg_impedance_controller, quad_com_control
from dynamic_graph.sot.core.reader import Reader

#############################################################################

def file_exists(filename):
    if os.path.isfile(filename):
        print("The file %s exists" % filename)
    else:
        print("The file %s does not exist" % filename)

#############################################################################
### reading createData
reader_pos = Reader('PositionReader')
reader_vel = Reader('VelocityReader')
reader_com = Reader('ComReader')

filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"
filename_vel = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_velocities_eff.dat"
filename_com = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_com.dat"
#filename_pos = "/home/ameduri/devel/kino-dynamic-opt/src/catkin/motion_planning/momentumopt/demos/quadruped_positions_eff.dat"

# filename_pos = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/quadruped_positions_eff_rearing.dat"
# filename_vel = "/home/ameduri/devel/workspace/src/catkin/robots/dg_blmc_robots/demos/quadruped/trajectories/quadruped_velocities_eff_rearing.dat"

file_exists(filename_pos)
file_exists(filename_vel)
file_exists(filename_com)

print("Loading data files:")
reader_pos.load(filename_pos)
reader_vel.load(filename_vel)
reader_com.load(filename_com)

# Specify which of the columns to select.
# NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
reader_pos.selec.value = '111111111111111111111111'
reader_vel.selec.value = '111111111111111111111111'
reader_com.selec.value = '1111'

des_pos = reader_pos.vector
des_vel = reader_vel.vector


################################################################################
des_fff = constVector([0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, (2.2*9.8)/4.0, 0.0, 0.0, 0.0],
                        "fff")

##For making gain input dynamic through terminal
add_kf = Add_of_double('kf')
add_kf.sin1.value = 0
### Change this value for different gains
add_kf.sin2.value = 0.0
kf = add_kf.sout

###############################################################################

def start_traj():
    reader_pos.rewind()
    reader_pos.vector.recompute(0)
    reader_vel.rewind()
    reader_vel.vector.recompute(0)
    reader_com.rewind()
    reader_com.vector.recompute(0)


##For making gain input dynamic through terminal
add_kp = Add_of_double('kp')
add_kp.sin1.value = 0
### Change this value for different gains
add_kp.sin2.value = 10.0
kp = add_kp.sout

##For making gain input dynamic through terminal
add_kd = Add_of_double('kd')
add_kd.sin1.value = 0
### Change this value for different gains
add_kd.sin2.value = 0.0
kd = add_kd.sout

quad_imp_ctrl = quad_leg_impedance_controller(robot)
control_torques = quad_imp_ctrl.return_control_torques(kp, des_pos, kd, des_vel, kf, des_fff)

plug(control_torques, robot.device.ctrl_joint_torques)


####################################### data logging ########################################

quad_com_ctrl = quad_com_control(robot)

quad_imp_ctrl.record_data()

robot.add_trace("PositionReader", "vector")
robot.add_ros_and_trace("PositionReader", "vector")

robot.add_trace("VelocityReader", "vector")
robot.add_ros_and_trace("VelocityReader", "vector")

robot.add_trace("ComReader", "vector")
robot.add_ros_and_trace("ComReader", "vector")
