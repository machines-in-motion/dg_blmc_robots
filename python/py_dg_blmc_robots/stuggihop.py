from __future__ import print_function

import os
import rospkg
import numpy as np
import time

import dynamic_graph_manager as dgm
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

import py_robot_properties_stuggihop
from py_robot_properties_stuggihop.config import StuggihopConfig

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from dynamic_graph.sot.core.vector_constant import VectorConstant

from py_pinocchio_bullet.wrapper import PinBulletWrapper

class StuggihopBulletRobot(Robot):
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)

        # Load the plain.
        plain_urdf = rospkg.RosPack().get_path("robot_properties_stuggihop") \
        + "/urdf/plane_with_restitution.urdf"

        self.planeId = p.loadURDF(plain_urdf)

        print("Loaded ground.")

        # Load the robot
        robotStartPos = [0.,0,0.40]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = StuggihopConfig.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos,
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
            useFixedBase=True)
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path))
        + '/urdf']
        self.pin_robot = StuggihopConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04,
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(1e-3, numSubSteps=1)

        # no base link, since we're using a fixed base
        # self.base_link_name = "base_link" 
        self.joint_names = ['joint_x', 'joint_z', 'HFE', 'KFE']

        # no controlled joints... not sure why. This is from stuggihop
        # controlled_joints = ['FL_HFE', 'FL_KFE', 'FR_HFE', 'FR_KFE',
        # 'HL_HFE', 'HL_KFE', 'HR_HFE', 'HR_KFE']

        self.wrapper = PinBulletWrapper(self.robotId, self.pin_robot,
            self.joint_names,['END'], useFixedBase=True)
            # controlled_joints, # this is fro stuggihop
            # ['HL_END', 'HR_END', 'FL_END', 'FR_END'])

        # Initialize the device.
        self.device = Device('bullet_stuggihop')
        self.device.initialize(StuggihopConfig.yaml_path)

        # # Create signals for the base.
        # self.signal_base_pos_ = VectorConstant("bullet_stuggihop_base_pos")
        # self.signal_base_vel_ = VectorConstant("bullet_stuggihop_base_vel")
        # self.signal_base_pos_.sout.value = np.hstack([robotStartPos,
        #     robotStartOrientation]).tolist()
        # self.signal_base_vel_.sout.value = [0., 0., 0., 0., 0., 0.]

        # Initialize signals that are not filled in sim2signals.
        #self.device.motor_encoder_indexes.value = 8 * [0.]
        # self.device.slider_positions.value = 1 * [0.]
        # self.device.contact_sensors.value = 1 * [0.]
        # self.device.height_sensors.value = 1 * [0.]

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()

        self.steps_ = 0

        super(StuggihopBulletRobot, self).__init__('bullet_stuggihop',
            self.device)

    def pinocchio_robot_wrapper(self):
        return self.pin_robot

    # def base_signals(self):
    #     return self.signal_base_pos_.sout, self.signal_base_vel_.sout

    def sim2signal_(self):
        """ Reads the state from the simulator 
        and fills the corresponding signals. """

        # TODO: Handle the following signals:
        # - joint_target_torques
        # - joint_torques

        q, dq = [np.array(t).reshape(-1).tolist() for t \
        in self.wrapper.get_state()]

        device = self.device
        device.joint_positions.value = q[2:]
        device.joint_velocities.value = dq[2:]
        # device.base_positions.value = q[0:1]
        # device.base_velocities.value = dq[0:1]
        # ?? Base values?
        

    def run(self, steps=1, delay=0.):
        tau = zero(self.wrapper.nv) # we need to skip the unactuated DoFs
        for i in range(steps):
            self.device.execute_graph()
            tau[2:] = np.matrix(self.device.ctrl_joint_torques.value).T
            self.wrapper.send_joint_command(tau)
            p.stepSimulation()
            self.sim2signal_()
            self.steps_ += 1

            # TODO: what is this magic nbr?
            if delay != 0. and self.steps_ % 17 == 0:
                time.sleep(delay)

    def reset_state(self, q, dq):
        """ Sets the bullet simulator and 
        the signals to the provided state values. """
        self.wrapper.reset_state(q, dq)
        self.sim2signal_()

    def set_gravity(self, vec):
        """ Sets gravity in the simulator to (x,y,z), where 
        z is the vertical axis. """
        p.setGravity(vec[0],vec[1], vec[2])

def get_stuggihop_robot():
    return StuggihopBulletRobot()
