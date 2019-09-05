## This is code to track desired trajectories from the planner
## The gains have to be tuned with the sliders
## and leg impedance

##Author : Elham

from leg_impedance_control.utils import *
from leg_impedance_control.traj_generators import mul_double_vec_2, scale_values
from leg_impedance_control.leg_impedance_controller import leg_impedance_controller
from dynamic_graph.sot.core.fir_filter import FIRFilter_Vector_double
from dynamic_graph.sot.core.reader import Reader
from os.path import join

class ComPlannedMotion:
    def __init__(self, robot):
        slider_filtered = FIRFilter_Vector_double("slider_fir_filter")
        self.filter_size = 400
        slider_filtered.setSize(self.filter_size)
        for i in range(self.filter_size):
            slider_filtered.setElement(i, 1.0/float(self.filter_size))

        # we plug the centered sliders output to the input of the filter.
        plug(robot.device.slider_positions, slider_filtered.sin)

        self.slider_1_op = Component_of_vector("slider_1")
        self.slider_1_op.setIndex(0)
        plug(slider_filtered.sout, self.slider_1_op.sin)
        self.slider_1 = self.slider_1_op.sout

        self.slider_2_op = Component_of_vector("slider_2")
        self.slider_2_op.setIndex(1)
        plug(slider_filtered.sout, self.slider_2_op.sin)
        self.slider_2 = self.slider_2_op.sout

        self.kp = scale_values(self.slider_1, 200.0, "scale_kp")#######
        #self.kd = scale_values(self.slider_2, 12.5, "scale_kd")########

        self.unit_vec_101 = constVector([1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "unit_vec_101")

        self.kp = mul_double_vec_2(self.kp, self.unit_vec_101, "kp")
        #self.kd = mul_double_vec_2(self.kd, self.unit_vec_101, "kd")
        self.kd = constVector([0.8, 0.0, 2.0, 0.0, 0.0, 0.0], "kd")

        ################################################################################

        def file_exists(filename):
            if os.path.isfile(filename):
                print("The file %s exists" % filename)
            else:
                print("The file %s does not exist" % filename)
                assert False

        self.reader_pos = Reader('PositionReader')
        self.reader_vel = Reader('VelocityReader')
        self.reader_fff_com = Reader('FeedForwardForceComReader')

        self.filename_pos = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_positions_eff.dat")
        self.filename_vel = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_velocities_eff.dat")
        self.filename_fff_com = join(rospkg.RosPack().get_path("momentumopt"),"demos","quadruped_centroidal_forces.dat")

        file_exists(self.filename_pos)
        file_exists(self.filename_vel)
        file_exists(self.filename_fff_com)

        print("Loading data files:")
        self.reader_pos.load(self.filename_pos)
        self.reader_vel.load(self.filename_vel)
        self.reader_fff_com.load(self.filename_fff_com)

        # Specify which of the columns to select.
        # NOTE: This is selecting the columns in reverse order - the last number is the first column in the file
        self.reader_pos.selec.value = '000000000000000000111111'
        self.reader_vel.selec.value = '000000000000000000111111'
        self.reader_fff_com.selec.value = '1110'

        self.des_pos = self.reader_pos.vector
        self.des_vel = self.reader_vel.vector
        #des_fff = reader_fff_com.vector

        ###############################################################################

        self.add_kf = Add_of_double('kf')
        self.add_kf.sin1.value = 0
        ### Change this value for different gains
        self.add_kf.sin2.value = 0.0
        self.kf = self.add_kf.sout

        leg_imp_ctrl = leg_impedance_controller("hopper")
        plug(stack_zero(robot.device.signal('joint_positions'), "add_base_joint_position"), leg_imp_ctrl.robot_dg.position)
        plug(stack_zero(robot.device.signal('joint_velocities'), "add_base_joint_velocity"), leg_imp_ctrl.robot_dg.velocity)

        control_torques = leg_imp_ctrl.return_control_torques(self.kp, self.des_pos, self.kd, self.des_vel, self.kf, None)

        plug(control_torques, robot.device.ctrl_joint_torques)

    def start_traj(self):
        self.reader_pos.rewind()
        self.reader_pos.vector.recompute(0)
        self.reader_vel.rewind()
        self.reader_vel.vector.recompute(0)
        self.reader_fff_com.rewind()
        self.reader_fff_com.vector.recompute(0)


if ('robot' in globals()) or ('robot' in locals()):
    planned_motion = ComPlannedMotion(robot)