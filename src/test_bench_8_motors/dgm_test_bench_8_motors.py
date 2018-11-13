#
# @file prologue.py
# @brief The robot entity in python
# @author Maximilien Naveau
# @date 2018
#
# This file prepares the pyhton interpretor so it contains a pointer to the
# Device and some tracers
#

def set_current(currents):
    if shape(currents) is not None:
        robot.device.signal("ctrl_motor_currents").value = currents

