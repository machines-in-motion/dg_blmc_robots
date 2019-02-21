from __future__ import print_function

import os
import rospkg
import numpy as np
import time

import dynamic_graph_manager as dgm
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

import py_robot_properties_teststand
from py_robot_properties_teststand.config import TeststandConfig

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from dynamic_graph.sot.core.vector_constant import VectorConstant

from py_pinocchio_bullet.wrapper import PinBulletWrapper

class TeststandBulletRobot(Robot):
    def __init__(self):
        self.physicsClient = p.connect(p.GUI)

        # Load the plain.
        plain_urdf = rospkg.RosPack().get_path("robot_properties_teststand") + \
        "/urdf/plane_with_restitution.urdf"
        self.planeId = p.loadURDF(plain_urdf)

        print("Loaded plain.")

        # Load the robot
        robotStartPos = [0., 0., 0.]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        self.urdf_path = TeststandConfig.urdf_path
        self.robotId = p.loadURDF(self.urdf_path, robotStartPos, 
            robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE, 
            useFixedBase=True)
        p.getBasePositionAndOrientation(self.robotId)

        # Create the robot wrapper in pinocchio.
        package_dirs = [os.path.dirname(os.path.dirname(self.urdf_path)) 
        + '/urdf']
        self.pin_robot = TeststandConfig.buildRobotWrapper()

        # Query all the joints.
        num_joints = p.getNumJoints(self.robotId)

        for ji in range(num_joints):
            p.changeDynamics(self.robotId, ji, linearDamping=.04, 
                angularDamping=0.04, restitution=0.0, lateralFriction=0.5)

        p.setGravity(0,0, -9.81)
        p.setPhysicsEngineParameter(1e-3, numSubSteps=1)

        self.joint_names = ['joint_z', 'HFE', 'KFE']

        self.wrapper = PinBulletWrapper(self.robotId, self.pin_robot, 
            self.joint_names, ['END'], useFixedBase=True)

        # Initialize the device.
        self.device = Device('bullet_teststand')
        self.device.initialize(TeststandConfig.yaml_path)

        # Initialize signals that are not filled in sim2signals.
        self.device.slider_positions.value = 1 * [0.]
        self.device.contact_sensors.value = 1 * [0.]
        self.device.height_sensors.value = 1 * [0.]
        self.device.ati_force.value = 3 * [0.]
        self.device.ati_torque.value = 3 * [0.]

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()

        self.steps_ = 0

        super(TeststandBulletRobot, self).__init__('bullet_teststand', 
            self.device)

    def pinocchio_robot_wrapper(self):
        return self.pin_robot

    def sim2signal_(self):
        """ Reads the state from the simulator and fills 
        the corresponding signals. """

        # TODO: Handle the following signals:
        # - joint_target_torques
        # - joint_torques

        q, dq = [np.array(t).reshape(-1).tolist() for t in 
        self.wrapper.get_state()]

        device = self.device
        device.joint_positions.value = q[1:]
        device.joint_velocities.value = dq[1:]

        device.height_sensors.value = [q[0]]

        ## uncomment if force at the ground is desired
        # #contact_frames, contact_forces = self.wrapper.get_force()
        # if (len(contact_frames) > 0):
        #     device.ati_force.value = (-contact_forces[0]).tolist()
        # else:
        #     device.ati_force.value = 3 * [0.]


    def run(self, steps=1, delay=0.):
        tau = zero(self.wrapper.nv)
        for i in range(steps):
            self.device.execute_graph()
            # The base is not actuated.
            tau[1:] = np.matrix(self.device.ctrl_joint_torques.value).T
            self.wrapper.send_joint_command(tau)
            p.stepSimulation()
            self.sim2signal_()
            self.steps_ += 1

            if delay != 0. and self.steps_ % 17 == 0:
                time.sleep(delay)

    def reset_state(self, q, dq):
        """ Sets the bullet simulator and the signals to 
        the provided state values. """
        self.wrapper.reset_state(q, dq)
        self.sim2signal_()


def get_teststand_robot():
    return TeststandBulletRobot()