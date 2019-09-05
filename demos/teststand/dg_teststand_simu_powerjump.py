## Simulation for power jump with teststand
## Author : Avadesh Meduri
## Date : 14/04/2019

from py_dg_blmc_robots.teststand import get_teststand_robot
from py_robot_properties_teststand.config import TeststandConfig

from dg_teststand_powerjump import PowerJump

# Get the robot corresponding to the quadruped.
robot = get_teststand_robot(fixed_slider=False)
config = TeststandConfig()

# Update the initial state of the robot.
robot.reset_state(config.q0, config.v0)

# Load the controller
ctrl = PowerJump(robot)

# Run the simulation
robot.run(10000, 1./60.)