## simple jump on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values, sine_generator

from dynamic_graph.sot.core.control_pd import ControlPD

from dynamic_graph_manager.dg_tools import Calibrator
from dynamic_graph.sot.core.switch import SwitchVector

#######################################################################################

######### CALIBRATOR SETUP #############
print("Starting up calibrator")
leg_calibrator = Calibrator('leg_calibrator')

from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
calib_vel_filtered = FIRFilter_Vector_double("calib_vel_filtered")
filter_size = 100
calib_vel_filtered.setSize(filter_size)
for i in range(filter_size): # probably a weighting
    calib_vel_filtered.setElement(i, 1.0/float(filter_size))
plug(robot.device.joint_velocities, calib_vel_filtered.sin)

plug(robot.device.joint_positions, leg_calibrator.raw_position)
plug(calib_vel_filtered.sout, leg_calibrator.velocity)

leg_calibrator.calibration_torque.value = (-0.2, 0.1) # 8*[0.0,]
# leg_calibrator.kp.value = 2*(0.3,)
leg_calibrator.hardstop2zero.value = 2*(0.0,)

######################################## Set up PD controller

pd = ControlPD("pd_ctrl")
pd.Kp.value = (0.0, 1.0)
pd.Kd.value = (0.0, 0.01)

pd.desiredposition.value = 2*(0.0,)
pd.desiredvelocity.value = 2*(0.0,)

plug(leg_calibrator.calibrated_position, pd.position)
plug(calib_vel_filtered.sout, pd.velocity)

##################### SWITCH between controllers

control_switch = SwitchVector("control_switch")
control_switch.setSignalNumber(2) # we want to switch between 2 signals
plug(leg_calibrator.control, control_switch.sin0)

# control_switch.sin1.value = (0.0, 0.0) # 8*[0.0,]
plug(pd.control, control_switch.sin1)

control_switch.selection.value = 0 # pick and switch manually for now
plug(control_switch.sout, robot.device.ctrl_joint_torques)

###################### Record Data ##########################################################

# leg_imp_ctrl.record_data(robot)

robot.add_trace("leg_calibrator", "calibrated_position")
robot.add_ros_and_trace("leg_calibrator", "calibrated_position")

robot.add_trace("calib_vel_filtered", "sout")
robot.add_ros_and_trace("calib_vel_filtered", "sout")

robot.add_trace("pd_ctrl", "control")
robot.add_ros_and_trace("pd_ctrl", "control")

# robot.add_trace("hopper_des_position", "sout")
# robot.add_ros_and_trace("hopper_des_position", "sout")

# robot.add_trace("hopper_des_velocity", "sout")
# robot.add_ros_and_trace("hopper_des_velocity", "sout")

# robot.add_trace("slider_fir_filter", "sout")
# robot.add_ros_and_trace("slider_fir_filter", "sout")
