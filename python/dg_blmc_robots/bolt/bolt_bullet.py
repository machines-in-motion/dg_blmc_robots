from __future__ import print_function

import os
import rospkg
import numpy as np
import time

import dynamic_graph_manager as dgm
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

from robot_properties_bolt.config import BoltConfig

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from matplotlib import pyplot as plt

from dynamic_graph.sot.core.vector_constant import VectorConstant

from py_pinocchio_bullet.wrapper import PinBulletWrapper

from dg_blmc_robots.bolt.bolt_base_bullet import BoltBaseRobot

class BoltBulletRobot(BoltBaseRobot):
    def __init__(self, use_fixed_base=False, record_video=False,
                 init_sliders_pose=4*[0.5]):

        super(BoltBulletRobot, self).__init__(BoltConfig(), use_fixed_base,
                record_video, init_sliders_pose)

        self.q0[2] = 0.26487417
        self.q0[6] = 1.
        self.q0[7] = 0.0
        self.q0[8] = -0.78539816
        self.q0[9] = 1.57079633
        self.q0[10] = 0.0
        self.q0[11] = -0.78539816
        self.q0[12] = 1.57079633

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()


def get_bolt_robot(use_fixed_base=False, record_video = False,
              init_sliders_pose=4*[0.5]):
    return BoltBulletRobot(use_fixed_base, record_video, init_sliders_pose)
