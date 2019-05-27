## simple jump on impact test stand
## Author : Avadesh Meduri
## Date : 1/03/19

from leg_impedance_control.utils import *
from leg_impedance_control.leg_impedance_controller import (
  leg_impedance_controller
)
from leg_impedance_control.traj_generators import (
  Multiply_double_vector, scale_values, sine_generator
)

## filter slider_value
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double


class StiffnessMeasurement:
    def __init__(self, robot, name="stiffness_measurement"):
        #
        # Get the arguments
        #
        self.name = name
        self.robot = robot

        #
        # Filter the input signals
        #
        # we define a filter for the height sensor, the output is our Height
        # sensor data
        self.height_sensor_filtered = FIRFilter_Vector_double(self.name + "_height_sensor")
        # initilialize the filter
        filter_size = 100
        self.height_sensor_filtered.setSize(filter_size)
        for i in range(filter_size):
            self.height_sensor_filtered.setElement(i, 1.0/float(filter_size))
        # we plug the hardware height_sensors signal into the filter input.
        plug(self.robot.device.height_sensors, self.height_sensor_filtered.sin)

        #
        # Defines the impedance controller parameters
        #

        # create a vector of "kp gains", i.e. the stiffness in the cartesian
        # space [X Y Z Wx Wy Wz]
        self.kp_const_vec = constVector([0.4, 0.0, 1.0, 0.0, 0.0, 0.0],
                                             self.name + "kp_const_vec")
        # multiply the kp_const_vec by a constant double to scale them
        self.kp_mult_double_vec = Multiply_double_vector(self.name + "kp_mult_double_vec")
        # this variable scales the kp_const_vec
        self.kp_mult_double_vec.sin1.value = 20.0
        # plug the const vector to the multiplication entity
        plug(self.kp_const_vec, self.kp_mult_double_vec.sin2)
        
        # some renaming for conveniency
        self.kp_scale = self.kp_mult_double_vec.sin1
        self.kp_gains = self.kp_mult_double_vec.sout

        # for the kd "damping" gains we set them directly
        self.kd_gains = constVector([0.8, 0.0, 2.0, 0.0, 0.0, 0.0],
                                    self.name + "kd_gains")

        # Define the gain on the feed forward term.
        # Create a sum of double in order to get a constant double.
        self.kf_add_double = Add_of_double(self.name + 'kf_add_double')
        self.kf_add_double.sin1.value = 0.0
        self.kf_add_double.sin2.value = 0.0
        # some renaming for conveniency
        self.kf_ratio = self.kf_add_double.sin2
        self.kf_ratio_out = self.kf_add_double.sout
        
        # the desired feed forward term:
        mass = 0.8
        gravity = 9.81
        self.weight = mass * gravity
        self.des_fff = constVector([0.0, 0.0, self.weight, 0.0, 0.0, 0.0],
                                   self.name + "des_fff")

        # Desired cartesian position of the foot compare to the 
        self.des_leg_length_pos = constVector([0.0, 0.0, -0.22, 0.0, 0.0, 0.0], "des_leg_length_pos")
        self.des_pos = self.des_leg_length_pos

        self.des_leg_length_vel = constVector([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "des_leg_length_vel")
        self.des_vel = self.des_leg_length_vel

        # #######################################################################################

        self.leg_imp_ctrl = leg_impedance_controller("hopper")

        plug(stack_zero(self.robot.device.signal('joint_positions'), "add_base_joint_position"), self.leg_imp_ctrl.robot_dg.position)
        plug(stack_zero(self.robot.device.signal('joint_velocities'), "add_base_joint_velocity"), self.leg_imp_ctrl.robot_dg.velocity)


        self.control_torques = self.leg_imp_ctrl.return_control_torques(
          self.kp_gains, self.des_pos,
          self.kd_gains, self.des_vel,
          self.kf_ratio_out, self.des_fff)

        plug(self.control_torques, self.robot.device.ctrl_joint_torques)

        # ###################### Record Data ##########################################################

        self.leg_imp_ctrl.record_data(self.robot)

        self.robot.add_ros_and_trace("des_leg_length_pos", "sout")

        self.robot.add_ros_and_trace("des_leg_length_vel", "sout")

        self.robot.add_ros_and_trace(self.name + "_height_sensor", "sout")


if ('robot' in globals()) or ('robot' in locals()):
    stiffness_meas = StiffnessMeasurement(robot)
