## power jump with teststand using a very stupid algorithm
## Author : Maximilien Naveau
## Date : 14/04/2019

import numpy as np
from dynamic_graph import plug, writeGraph
from dynamic_graph.sot.tools import Oscillator
from dynamic_graph.sot.core.vector_constant import VectorConstant
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.operator import (
  Multiply_double_vector,
  Add_of_double,
  Component_of_vector,
  Selec_of_vector,
  Stack_of_vector,
  Substract_of_vector,
)

class JointOscillation:
    def __init__(self, robot, name="joint_oscillation"):
        self.name = name
        self.robot = robot

        self.magnitude_maximum = 0.8 # rad
        self.pulsation_maximum = 3.0 * 2.0 * np.pi # rad/second
        self.initial_hip_angle = 0.8 # see config.py

        # Filter the centered sliders
        # Hence we create a "Finite Impendance Response" filter.
        # the filter is in the following form:
        # out = sum_{i=0}^{N} data_i * alpha_i
        #   - the data_i are the collected elements, their number grows until the
        #     size of the filter is reached.
        #   - the alpha_i are the gains of the filter, they are defined by the
        #     method "setElement(index, value)"
        # in the end here we do an averaging filter on 200 points.
        self.slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
        filter_size = 200
        self.slider_filtered.setSize(filter_size)
        for i in range(filter_size):
            self.slider_filtered.setElement(i, 1.0/float(filter_size))
        # we plug the centered sliders output to the input of the filter.
        plug(robot.device.slider_positions, self.slider_filtered.sin)

        # select the slider A and B
        self.slider_a = Selec_of_vector(name + "_slider_a")
        self.slider_a.selec(0, 1)
        plug(self.slider_filtered.sout, self.slider_a.sin)
        #
        self.slider_b = Selec_of_vector(name + "_slider_b")
        self.slider_b.selec(1, 2)
        plug(self.slider_filtered.sout, self.slider_b.sin)
        
        # Now we want the slider A to be in [-amp_max, amp_max] and B in [0, freq_max]
        self.magnitude_vec = Multiply_double_vector(name + "_magnitude_vec")
        self.magnitude_vec.sin1.value = self.magnitude_maximum
        plug(self.slider_a.sout, self.magnitude_vec.sin2)
        self.magnitude = Component_of_vector(name + "_magnitude")
        self.magnitude.setIndex(0)
        plug(self.magnitude_vec.sout, self.magnitude.sin)
        #
        self.pulsation_vec = Multiply_double_vector(name + "_pulsation_vec")
        self.pulsation_vec.sin1.value = self.pulsation_maximum
        plug(self.slider_b.sout, self.pulsation_vec.sin2)
        self.pulsation = Component_of_vector(name + "_pulsation")
        self.pulsation.setIndex(0)
        plug(self.pulsation_vec.sout, self.pulsation.sin)
        
        # phase
        self.phase = Add_of_double(name + "_phase")
        self.phase.sin1.value = 0
        self.phase.sin2.value = 0

        # bias
        self.bias = Add_of_double(name + "_bias")
        self.bias.sin1.value = 0
        self.bias.sin2.value = self.initial_hip_angle

        # pi
        self.half_pi = Add_of_double(name + "_pi")
        self.half_pi.sin1.value = 0
        self.half_pi.sin2.value = np.pi/2.0

        ## generates a "hip_pos = a*sin(W.t + phi)"
        self.hip_pos = Oscillator(name + "_hip_pos")
        self.hip_pos.setTimePeriod(0.001)
        plug(self.pulsation.sout, self.hip_pos.omega)
        plug(self.magnitude.sout, self.hip_pos.magnitude)
        plug(self.phase.sout, self.hip_pos.phase)
        plug(self.bias.sout, self.hip_pos.bias)
        self.hip_pos.setActivated(True)
        # self.hip_pos.setContinuous(True)
        # generate the derivation of this signal
        self.hip_vel = Oscillator(name + '_hip_vel')
        self.hip_vel.setTimePeriod(0.001)
        plug(self.pulsation.sout, self.hip_vel.omega)
        plug(self.magnitude.sout, self.hip_vel.magnitude)
        self.shift_phase = Add_of_double(name + "_shift_phase")
        plug(self.phase.sout, self.shift_phase.sin1)
        plug(self.half_pi.sout, self.shift_phase.sin2)
        plug(self.shift_phase.sout, self.hip_vel.phase )
        self.hip_vel.bias.value = 0
        self.hip_vel.setActivated(True)
        # self.hip_vel.setContinuous(True)

        # Knee = -2 * hip
        self.knee_pos = Multiply_double_vector(name + "_knee_pos")
        self.knee_pos.sin1.value = -2.0
        plug(self.hip_pos.vectorSout, self.knee_pos.sin2)
        #
        self.knee_vel = Multiply_double_vector(name + "_knee_vel")
        self.knee_vel.sin1.value = -2.0
        plug(self.hip_vel.vectorSout, self.knee_vel.sin2)

        # desired_position
        self.desired_position = Stack_of_vector(name + "_desired_position")
        self.desired_position.selec1(0, 1)
        self.desired_position.selec2(0, 1)
        plug(self.hip_pos.vectorSout, self.desired_position.sin1)
        plug(self.knee_pos.sout, self.desired_position.sin2)

        # desired_velocity
        self.desired_velocity = Stack_of_vector(name + "_desired_velocity")
        self.desired_velocity.selec1(0, 1)
        self.desired_velocity.selec2(0, 1)
        plug(self.hip_vel.vectorSout, self.desired_velocity.sin1)
        plug(self.knee_vel.sout, self.desired_velocity.sin2)

        # create the pid control
        self.pd_ctrl = ControlPD(name + "_pd_ctrl")
        # reset and redimension all input signals:
        self.pd_ctrl.Kp.value = 2 * [5.0,]
        self.pd_ctrl.Kd.value = 2 * [0.1,]
        plug(robot.device.joint_positions, self.pd_ctrl.position)
        plug(self.desired_position.sout, self.pd_ctrl.desired_position)
        plug(robot.device.joint_velocities, self.pd_ctrl.velocity)
        plug(self.desired_velocity.sout, self.pd_ctrl.desired_velocity)

        # Plug the control to the robot
        plug(self.pd_ctrl.control, self.robot.device.ctrl_joint_torques)

        # dump the graph in /tmp
        writeGraph('/tmp/teststand_joint_oscillation.dot')

        # add some ros debug:
        robot.add_to_ros(name + "_hip_pos", "omega", topic_type="double")
        robot.add_to_ros(name + "_hip_pos", "magnitude", topic_type="double")
        robot.add_to_ros(name + "_hip_pos", "phase", topic_type="double")
        robot.add_to_ros(name + "_hip_pos", "bias", topic_type="double")
        robot.add_to_ros(name + "_hip_pos", "sout", topic_type="double")

if ('robot' in globals()) or ('robot' in locals()):
    joint_oscillator = JointOscillation(robot)
