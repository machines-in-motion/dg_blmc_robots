## Example for the Calibrator
## Author : Steve Heim
## Date : 27/03/19
#
# Starts calibration by driving joints with constant torque into the hardstops
# (or your hands). It will then save the offsets and switch off the motors.
# You can then set the `control_switch.selection.value` to `1`, in order to turn
# on a PD control to hold a 0 position. Replace control_switch.sin1 with your 
# own controller.
#
# How does the calibrator work? the calibrator will subtract/add 2 values to
# each position: 
# start2hardstop: ths value is calibrated each time
# hardstop2zero: this is a fixed value, which you need to assign to the SIN
#
# You also need to specify reasonable constant-torques to drive each joint. They
# should be low enough to drive the joints softly into their hardstops.
# The calibrator will ramp from 0 to the torque you specify in 
# 1 second (1000 timesteps). After 1 second, it will check if the velocity is
# below a small threshold to determine if the hardstop has been reached.
# Each joint is turned off right after.
#
# It is YOUR JOB to:
# - make sure you use sensible torques
# - drive the joints in the correct direction
# - provide the hardstop2zero offsets
# - drive to a zero position after calibration

from dynamic_graph import plug

from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph_manager.dg_tools import Calibrator
from dynamic_graph.sot.core.switch import SwitchVector

################################################################################

######### CALIBRATOR SETUP #############
print("Starting up calibrator")
leg_calibrator = Calibrator('leg_calibrator')

# let's use a filtered velocity (optional)
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
calib_vel_filtered = FIRFilter_Vector_double("calib_vel_filtered")
filter_size = 100
calib_vel_filtered.setSize(filter_size)
for i in range(filter_size): # probably a weighting
    calib_vel_filtered.setElement(i, 1.0/float(filter_size))
plug(robot.device.joint_velocities, calib_vel_filtered.sin)

plug(robot.device.joint_positions, leg_calibrator.raw_position)
plug(calib_vel_filtered.sout, leg_calibrator.velocity)

# leg_calibrator.calibration_torque.value = 4*(-0.1, 0.1) # 8*[0.0,]
leg_calibrator.calibration_torque.value = (0.0,0.1, 0.0,0.0, 0.0,0.0, 0.0,0.0)

leg_calibrator.hardstop2zero.value = 8*(0.0,)

######################################## Set up PD controller

pd = ControlPD("pd_ctrl")
pd.Kp.value = 4*(1.0, 1.0)
pd.Kd.value = 4*(0.02, 0.02)

pd.desiredposition.value = 8*(0.0,)
pd.desiredvelocity.value = 8*(0.0,)

plug(leg_calibrator.calibrated_position, pd.position)
plug(calib_vel_filtered.sout, pd.velocity)

##################### SWITCH between controllers

control_switch = SwitchVector("control_switch")
control_switch.setSignalNumber(2) # we want to switch between 2 signals
plug(leg_calibrator.control, control_switch.sin0)
plug(pd.control, control_switch.sin1)
control_switch.selection.value = 0 # pick and switch manually

plug(control_switch.sout, robot.device.ctrl_joint_torques)

###################### Record Data #############################################

robot.add_trace("leg_calibrator", "calibrated_position")
robot.add_ros_and_trace("leg_calibrator", "calibrated_position")

robot.add_trace("leg_calibrator", "control")
robot.add_ros_and_trace("leg_calibrator", "control")

robot.add_trace("calib_vel_filtered", "sout")
robot.add_ros_and_trace("calib_vel_filtered", "sout")

robot.add_trace("pd_ctrl", "control")
robot.add_ros_and_trace("pd_ctrl", "control")