## Author : Elham
# from dynamic_graph_manager.device.robot import robot

from leg_impedance_control.utils import *
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from leg_impedance_control.utils import *
from leg_impedance_control.traj_generators import circular_trajectory_generator, mul_doub_doub


def read_slider(number):
    slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 100
    slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        slider_filtered.setElement(i, 1.0 / float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(robot.device.slider_positions, slider_filtered.sin)
    slider_op = []
    slider = []
    for i in range(number):
        slider_op.append(Component_of_vector("slider_" + str(i)))
        slider_op[i].setIndex(i)
        plug(slider_filtered.sout, slider_op[i].sin)
        slider.append(slider_op[i].sout)
    return slider


slider = read_slider(2)

add_constant = Add_of_double('constant')
add_constant.sin1.value = 0.0
### Change this value for different gains
add_constant.sin2.value = -0.2
constant = add_constant.sout
radius = mul_doub_doub(constant, slider[0], "multiply")

leg_imp_ctrl = leg_impedance_controller("hopper")
plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)

add_phase = Add_of_double('phase')
add_phase.sin1.value = 0.0
### Change this value for different gains
add_phase.sin2.value = 0.0
phase = add_phase.sout

des_pos, des_vel = circular_trajectory_generator(radius, radius, slider[1], phase, -0.20, "circle")

kp_split = constVector([30.0, 0.0, 30.0, 0.0, 0.0, 0.0], "kp_split")
kd_split = constVector([0.8, 0.0, 2.0, 0.0, 0.0, 0.0], "kd_split")

result = leg_imp_ctrl.return_control_torques(kp_split, des_pos, None, des_vel)
plug(result, robot.device.ctrl_joint_torques)
