from __future__ import print_function

import os
import rospkg
import numpy as np
import time

from dynamic_graph_manager.dynamic_graph.device import Device
from dynamic_graph_manager.robot import Robot

from robot_properties_solo.config import SoloConfig

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from matplotlib import pyplot as plt

from dynamic_graph.sot.core.vector_constant import VectorConstant

from py_pinocchio_bullet.wrapper import PinBulletWrapper

class SoloBaseRobot(Robot):
    """
    Base implementation for solo8 and solo12 robot.
    """
    def __init__(self, solo_config, use_fixed_base=False, record_video=False,
                 init_sliders_pose=4*[0.5]):
        self.config = solo_config
        self.record_video = record_video
        self.physicsClient = p.connect(p.GUI)
        p.resetDebugVisualizerCamera(1.2, 50, -35, (0., 0., 0.))

        # Load the plain.
        plain_urdf = (rospkg.RosPack().get_path("robot_properties_solo") +
                      "/urdf/plane_with_restitution.urdf")
        self.planeId = p.loadURDF(plain_urdf)

        print("Loaded plain.")

        # Load the robot
        robotStartPos = [0.,0,.7]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = self.config.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=use_fixed_base)
        p.getBasePositionAndOrientation(self.robotId)

        

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) +
                       '/urdf']
        self.pin_robot = self.config.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=1.0/1000.0, numSubSteps=1)

        self.base_link_name = "base_link"
        self.joint_names = self.config.joint_names
        self.end_effector_names = self.config.end_effector_names

        self.wrapper = PinBulletWrapper(self.robotId, self.pin_robot,
            self.joint_names, self.end_effector_names
        )

        self.slider_a = p.addUserDebugParameter("a", 0, 1, init_sliders_pose[0])
        self.slider_b = p.addUserDebugParameter("b", 0, 1, init_sliders_pose[1])
        self.slider_c = p.addUserDebugParameter("c", 0, 1, init_sliders_pose[2])
        self.slider_d = p.addUserDebugParameter("d", 0, 1, init_sliders_pose[3])

        self.hl_index = self.pin_robot.model.getFrameId('HL_ANKLE')
        self.hr_index = self.pin_robot.model.getFrameId('HR_ANKLE')
        self.fl_index = self.pin_robot.model.getFrameId('FL_ANKLE')
        self.fr_index = self.pin_robot.model.getFrameId('FR_ANKLE')

        # Initialize the device.
        self.device = Device('bullet_quadruped')
        self.device.initialize(self.config.yaml_path)

        # Create signals for the base.
        self.signal_base_pos_ = VectorConstant("bullet_quadruped_base_pos")
        self.signal_base_vel_ = VectorConstant("bullet_quadruped_base_vel")
        self.signal_base_vel_world_ = VectorConstant("bullet_quadruped_base_vel_world")
        self.signal_base_pos_.sout.value = np.hstack([robotStartPos,
            robotStartOrientation]).tolist()
        self.signal_base_vel_.sout.value = [0., 0., 0., 0., 0., 0.]
        self.signal_base_vel_world_.sout.value = [0., 0., 0., 0., 0., 0.]

        # Initialize signals that are not filled in sim2signals.
        #self.device.motor_encoder_indexes.value = 8 * [0.]
        self.device.slider_positions.value = 4 * [0.]

        if hasattr(self.device, 'contact_sensors'):
            self.device.contact_sensors.value = 4 * [0.]

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()

        self.q0 = zero(self.pin_robot.nq)
        self.dq0 = zero(self.pin_robot.nv)

        self.steps_ = 0

        super(SoloBaseRobot, self).__init__('bullet_quadruped',
            self.device)

    def start_video_recording(self):
        if self.record_video:
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/tmp/bullet_quadruped.mp4")

    def stop_video_recording(self):
        if self.record_video:
            p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)

    def pinocchio_robot_wrapper(self):
        return self.pin_robot

    def base_signals(self):
        return self.signal_base_pos_.sout, self.signal_base_vel_.sout

    def sim2signal_(self):
        """ Reads the state from the simulator and fills
        the corresponding signals. """

        # TODO: Handle the following signals:
        # - joint_target_torques
        # - joint_torques

        q, dq = [np.array(t).reshape(-1).tolist() for t in
        self.wrapper.get_state()]

        device = self.device
        device.joint_positions.value = q[7:]
        device.joint_velocities.value = dq[6:]

        self.signal_base_pos_.sout.value = q[0:7]
        self.signal_base_vel_.sout.value = dq[0:6]
        self.signal_base_vel_world_.sout.value = self.wrapper.get_base_velocity_world().reshape(-1).tolist()

        device.slider_positions.value = [
          p.readUserDebugParameter(self.slider_a),
          p.readUserDebugParameter(self.slider_b),
          p.readUserDebugParameter(self.slider_c),
          p.readUserDebugParameter(self.slider_d),
        ]


    def run(self, steps=1, delay=0., plot = False):
        """
        Executes the simulation for n `steps`. To slow down the simulation
        steps for visualization, applies `delay` ms of waiting after performing
        17 (~= 1000/60) simulation steps. This allows to adjust the timing for
        visualization at 60 Hz.
        """

        tracked_base_pos = np.zeros((steps, 7))
        tracked_base_vel = np.zeros((steps, 6))

        for i in range(steps):
            self.device.execute_graph()
            self.wrapper.send_joint_command(np.matrix(
                self.device.ctrl_joint_torques.value).T)
            p.stepSimulation()
            self.sim2signal_()
            self.steps_ += 1

            if plot:
                tracked_base_pos[i] = self.signal_base_pos_.sout.value
                tracked_base_vel[i] = self.signal_base_vel_.sout.value

            if delay != 0. and self.steps_ % 17 == 0:
                time.sleep(delay)


        if plot:
            tracked_base_pos = np.asarray(tracked_base_pos)
            tracked_base_vel = np.asarray(tracked_base_vel)
            print(np.shape(tracked_base_pos))

            fig1, ax1 = plt.subplots(3,1,sharex = True)

            ax1[0].plot(tracked_base_pos[: ,0], color = "red", label = "base_pos_x")
            #ax1[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_fl_x")
            ax1[0].legend()
            ax1[0].set_xlabel("millisec")
            ax1[0].set_ylabel("m")
            ax1[0].grid()

            ax1[1].plot(tracked_base_pos[: ,1], color = "red", label = "base_pos_y")
            # ax1[1].plot(des_pos_fl[: ,1], color = "black", label = "des_pos_fl_z")
            ax1[1].legend()
            ax1[1].set_xlabel("millisec")
            ax1[1].set_ylabel("m")
            ax1[1].grid()

            ax1[2].plot(tracked_base_pos[: ,2], color = "red", label = "base_pos_z")
            # ax1[1].plot(des_pos_fl[: ,1], color = "black", label = "des_pos_fl_z")
            ax1[2].legend()
            ax1[2].set_xlabel("millisec")
            ax1[2].set_ylabel("m")
            ax1[2].grid()

            fig2, ax2 = plt.subplots(3,1,sharex = True)

            ax2[0].plot(tracked_base_vel[: ,0], color = "red", label = "base_vel_x")
            #ax1[0].plot(des_pos_fl[:, 0], color = "black", label = "des_pos_fl_x")
            ax2[0].legend()
            ax2[0].set_xlabel("millisec")
            ax2[0].set_ylabel("m/s")
            ax2[0].grid()

            ax2[1].plot(tracked_base_vel[: ,1], color = "red", label = "base_vel_y")
            # ax1[1].plot(des_pos_fl[: ,1], color = "black", label = "des_pos_fl_z")
            ax2[1].legend()
            ax2[1].set_xlabel("millisec")
            ax2[1].set_ylabel("m/s")
            ax2[1].grid()

            ax2[2].plot(tracked_base_vel[: ,2], color = "red", label = "base_vel_z")
            # ax1[1].plot(des_pos_fl[: ,1], color = "black", label = "des_pos_fl_z")
            ax2[2].legend()
            ax2[2].set_xlabel("millisec")
            ax2[2].set_ylabel("m/s")
            ax2[2].grid()


            plt.show()

    def reset_state(self, q, dq):
        """ Sets the bullet simulator and the signals to
        the provided state values. """
        self.wrapper.reset_state(q, dq)
        self.sim2signal_()

    def set_gravity(self, vec):
        """ Sets gravity in the simulator to (x,y,z), where z is
        the vertical axis. """
        p.setGravity(vec[0],vec[1], vec[2])

    def print_physics_engine_params(self):
        params = p.getPhysicsEngineParameters(self.physicsClient)
        print ("physics_engine_params:")
        for key in params:
            print("    - ", key, ": ", params[key])

    def print_physics_params(self):
        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
              (mass,
               lateral_friction,
               local_inertia_diag,
               local_inertia_pos,
               local_inertia_ori,
               resitution,
               rolling_friction,
               spinning_friction,
               contact_damping,
               contact_stiffness) = p.getDynamicsInfo(
                  bodyUniqueId=self.robotId,
                  linkIndex=ji)
              # for el in dynamics_info:
              #     print(el)
              print ("link ", ji)
              print ("    - mass : " , mass)
              print ("    - lateral_friction : " , lateral_friction)
              print ("    - local_inertia_diag : " , local_inertia_diag)
              print ("    - local_inertia_pos : " , local_inertia_pos)
              print ("    - local_inertia_ori : " , local_inertia_ori)
              print ("    - resitution : " , resitution)
              print ("    - rolling_friction : " , rolling_friction)
              print ("    - spinning_friction : " , spinning_friction)
              print ("    - contact_damping : " , contact_damping)
              print ("    - contact_stiffness : " , contact_stiffness)

    def add_ros_and_trace(self, client_name, signal_name, topic_name=None, topic_type=None):

        ## for vicon entity
        self.signal_name = signal_name
