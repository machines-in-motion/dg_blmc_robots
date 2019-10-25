## This code is a simple joint controller for the quadruped

## Author: Maximilien Naveau
## Date: 12/02./2019

from dynamic_graph import plug
from dynamic_graph.sot.core.control_pd import ControlPD
from dynamic_graph.sot.core.operator import (Multiply_double_vector,
                                             Selec_of_vector, Stack_of_vector,
                                             Substract_of_vector)
from dynamic_graph.sot.core.switch import SwitchVector
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from leg_impedance_control.traj_generators import cubic_interpolator
from leg_impedance_control.utils import constVector
from robot_properties_solo.config import SoloConfig

##########################################################################################
# Get the robot corresponding to the quadruped.
# from dg_blmc_robots.solo.solo_bullet import get_quadruped_robot
# robot = get_quadruped_robot()


#########################################################################################

def joint_pid_control(q, q_des, dq, dq_des, Kp, Kd, entity_name):
    """
    This defines a simple pd control
    """
    pid = ControlPD(entity_name)
    plug(q, pid.position)
    plug(q_des, pid.desired_position)
    plug(dq, pid.velocity)
    plug(dq_des, pid.desired_velocity)
    plug(Kp, pid.Kp)
    plug(Kd, pid.Kd)
    return pid

def get_q_ref_from_4silders(sliders, max_hip_joint, entity_name):
    """
    In this part we define the desired joint position the joints must reach from
    analogue input. These inputs are 4 sliders (linear potentiometers).
    Because we have only 4 data for 8 joints we will have decompose the signal
    in 4 different signal and then concatenate them to get a vector of dim 8.
    """

    class QRef:
        def __init__(self, entity_name):
            self.name = entity_name

    q_ref = QRef("q_ref")

    # Center the sliders. The value of the sliders is in [0, 1].
    # in our case we prefer [-0.5, 0.5]. In order to do this we use this
    # Substract_of_vector entity that will subrstract sin2 from sin1
    # We substract 0.5 to all element of the sliders.
    q_ref.centered_slider = Substract_of_vector("centered_slider")
    plug(sliders, q_ref.centered_slider.sin1)
    q_ref.centered_slider.sin2.value = [0.5, 0.5, 0.5, 0.5]

    # Filter the centered sliders
    # Hence we create a "Finite Impendance Response" filter.
    # the filter is in the following form:
    # out = sum_{i=0}^{N} data_i * alpha_i
    #   - the data_i are the collected elements, their number grows until the
    #     size of the filter is reached.
    #   - the alpha_i are the gains of the filter, they are defined by the
    #     method "setElement(index, value)"
    # in the end here we do an averaging filter on 200 points.
    q_ref.slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
    filter_size = 200
    q_ref.slider_filtered.setSize(filter_size)
    for i in range(filter_size):
        q_ref.slider_filtered.setElement(i, 1.0/float(filter_size))
    # we plug the centered sliders output to the input of the filter.
    plug(q_ref.centered_slider.sout, q_ref.slider_filtered.sin)

    # Now we want the slider to be in [-qref, qref]
    # So we multiply all sliders by a constant which is max_qref.
    q_ref.scaled_slider = Multiply_double_vector("scaled_slider")
    q_ref.scaled_slider.sin1.value = max_hip_joint
    plug(q_ref.slider_filtered.sout, q_ref.scaled_slider.sin2)

    # Now we need to solve the problem that we have 4 sliders for 8 motors.
    # Hence we will map 1 slider to the fron leg, and 1 to the hind leg.
    
    for i, leg in enumerate(["front", "hind"]):
        # first of all we define the references for the hip joint:
        q_ref.__dict__[leg + "_hip_qref"] = Selec_of_vector(leg + "_hip_qref")
        q_ref.__dict__[leg + "_hip_qref"].selec(i, i+1)
        plug(q_ref.scaled_slider.sout, q_ref.__dict__[leg + "_hip_qref"].sin)

        # Then we define the reference for the knee joint. We want the knee to move
        # twice as much as the hip and on the opposite direction
        q_ref.__dict__[leg + "_knee_qref"] = Multiply_double_vector(
            leg + "_knee_qref")
        q_ref.__dict__[leg + "_knee_qref"].sin1.value = - 2.0
        plug(q_ref.__dict__[leg + "_hip_qref"].sout,
             q_ref.__dict__[leg + "_knee_qref"].sin2)
      
        # now we need to stack the signals 2 by 2:
        q_ref.__dict__[leg + "_qref"] = Stack_of_vector(leg + "_qref")
        q_ref.__dict__[leg + "_qref"].selec1(0, 1)
        q_ref.__dict__[leg + "_qref"].selec2(0, 1)
        # first element is the hip
        plug(q_ref.__dict__[leg + "_hip_qref"].sout,
             q_ref.__dict__[leg + "_qref"].sin1)
        # second element is the knee
        plug(q_ref.__dict__[leg + "_knee_qref"].sout,
             q_ref.__dict__[leg + "_qref"].sin2)
    
    # We stack fr and hr legs together
    q_ref.fl_fr_qref = Stack_of_vector("fl_fr_qref")
    q_ref.fl_fr_qref.selec1(0, 2)
    q_ref.fl_fr_qref.selec2(0, 2)
    plug(q_ref.front_qref.sout, q_ref.fl_fr_qref.sin1)
    plug(q_ref.front_qref.sout, q_ref.fl_fr_qref.sin2)
    # We stack hl and fl legs together
    q_ref.hr_hl_qref = Stack_of_vector("hr_hl_qref")
    q_ref.hr_hl_qref.selec1(0, 2)
    q_ref.hr_hl_qref.selec2(0, 2)
    plug(q_ref.hind_qref.sout, q_ref.hr_hl_qref.sin1)
    plug(q_ref.hind_qref.sout, q_ref.hr_hl_qref.sin2)
    # Now we can stack the four leg together
    q_ref.q_ref = Stack_of_vector("q_ref")
    q_ref.q_ref.selec1(0, 4)
    q_ref.q_ref.selec2(0, 4)
    plug(q_ref.fl_fr_qref.sout, q_ref.q_ref.sin1)
    plug(q_ref.hr_hl_qref.sout, q_ref.q_ref.sin2)
    
    # Here we just do a simple shortcut
    q_ref.sout = q_ref.q_ref.sout
    return q_ref


def get_q_ref_from_interpolation(q_signal, max_hip_joint, entity_name):
    """
    In this part we define the desired joint positions from interpolation.
    These inputs are interpolator from 0 to 1.
    Because we have only 4 data for 8 joints we will have decompose the signal
    in 4 different signal and then concatenate them to get a vector of dim 8.
    """

    class QRef:
        def __init__(self, entity_name):
            self.name = entity_name

    name = "q_ref"
    q_ref = QRef(name)

    # Create the interpolator for going to zero:
    q_ref.q_zero = len(q_signal.value) * [0.0]
    q_ref.go0 = cubic_interpolator(q_signal, q_ref.q_zero, name + "_go0")

    # duration of the motion
    q_ref.duration = 2.0

    # Create the way points:
    #  _
    # < <
    q_ref.q_knee_back = [
      + max_hip_joint, -2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
    ]
    q_ref.go_knee_back = cubic_interpolator(
      q_signal, q_ref.q_knee_back, q_ref.duration, name + "_go_knee_back")
    #  _
    # > >
    q_ref.q_knee_front = [
      - max_hip_joint, +2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
    ]
    q_ref.go_knee_front = cubic_interpolator(
      q_signal, q_ref.q_knee_front, q_ref.duration, name + "_go_knee_front")
    #  _
    # > <
    q_ref.q_knee_inward = [
      - max_hip_joint, +2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
    ]
    q_ref.go_knee_inward = cubic_interpolator(
      q_signal, q_ref.q_knee_inward, q_ref.duration, name + "_go_knee_inward")
    #  _
    # < >
    q_ref.q_knee_outward = [
      + max_hip_joint, -2.0*max_hip_joint,
      + max_hip_joint, -2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
      - max_hip_joint, +2.0*max_hip_joint,
    ]
    q_ref.go_knee_outward = cubic_interpolator(
      q_signal, q_ref.q_knee_outward, q_ref.duration, name + "_go_knee_outward")

    q_ref.switch = SwitchVector(name + "_switch")
    q_ref.switch.setSignalNumber(4)
    q_ref.switch.selection.value = 0

    plug(q_ref.go_knee_back.sout, q_ref.switch.sin0)
    plug(q_ref.go_knee_front.sout, q_ref.switch.sin1)
    plug(q_ref.go_knee_inward.sout, q_ref.switch.sin2)
    plug(q_ref.go_knee_outward.sout, q_ref.switch.sin3)

    # Here we just do a simple shortcut
    q_ref.sout = q_ref.switch.sout
    return q_ref


def reset_q_ref_interpolation(q_ref):
    q_ref.go_knee_back.reset()
    q_ref.go_knee_front.reset()
    q_ref.go_knee_inward.reset()
    q_ref.go_knee_outward.reset()


def start_q_ref_interpolation(q_ref):
    q_ref.go_knee_back.start(q_ref.duration)
    q_ref.go_knee_front.start(q_ref.duration)
    q_ref.go_knee_inward.start(q_ref.duration)
    q_ref.go_knee_outward.start(q_ref.duration)


def get_joint_controller(robot):
    class JointController():
        def __init__(self):
            self.name = "leg_reverse"

    jt_ctrl = JointController()

    jt_ctrl.solo_config = SoloConfig()
    jt_ctrl.Kp = constVector(8*[jt_ctrl.solo_config.kp], jt_ctrl.name + "_Kp")
    jt_ctrl.Kd = constVector(8*[jt_ctrl.solo_config.kd], jt_ctrl.name + "_Kd")
    
    # jt_ctrl.q_ref = get_q_ref_from_4silders(
    #     robot.device.slider_positions, 2.0, jt_ctrl.name + "_q_ref")
    jt_ctrl.q_ref = get_q_ref_from_interpolation(
          robot.device.joint_positions, 2.0, jt_ctrl.name + "_q_ref" )

    jt_ctrl.dq_ref = constVector(8*[0.0], jt_ctrl.name + "_dq_ref")
    
    q = robot.device.joint_positions
    q_ref = jt_ctrl.q_ref.sout
    dq = robot.device.joint_velocities
    dq_ref = jt_ctrl.dq_ref
    Kp = jt_ctrl.Kp
    Kd = jt_ctrl.Kd
    
    jt_ctrl.pid = joint_pid_control(q, q_ref, dq, dq_ref, Kp, Kd, jt_ctrl.name)

    plug(jt_ctrl.pid.control, robot.device.ctrl_joint_torques)

    return jt_ctrl

motion_index = -1
def start_next(ctrl):
    """
    Here we define a sequencer
    """
    # make sure we actually do not input a crazy number
    global motion_index
    motion_index = motion_index + 1
    if motion_index >= 4 or motion_index < 0:
        motion_index=0
       
    reset_q_ref_interpolation(ctrl.q_ref)
    ctrl.q_ref.switch.selection.value = motion_index
    start_q_ref_interpolation(ctrl.q_ref)
    

if 'robot' in globals():
    ctrl = get_joint_controller(robot)