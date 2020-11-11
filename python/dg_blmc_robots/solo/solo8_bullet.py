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

from dg_blmc_robots.solo.solo_base_bullet import SoloBaseRobot

class QuadrupedBulletRobot(SoloBaseRobot):
    def __init__(self, use_fixed_base=False, record_video=False,
                 init_sliders_pose=4*[0.5]):

        super(QuadrupedBulletRobot, self).__init__(SoloConfig(), use_fixed_base,
                record_video, init_sliders_pose)

        self.q0[0] = 0.2
        self.q0[1] = 0.0
        self.q0[2] = 0.22
        self.q0[6] = 1.
        self.q0[7] = 0.8
        self.q0[8] = -1.6
        self.q0[9] = 0.8
        self.q0[10] = -1.6
        self.q0[11] = -0.8
        self.q0[12] = 1.6
        self.q0[13] = -0.8
        self.q0[14] = 1.6

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()


def get_robot(use_fixed_base=False, record_video = False,
              init_sliders_pose=4*[0.5], with_gui=True):
    return QuadrupedBulletRobot(use_fixed_base, record_video, init_sliders_pose)


# Alias to new solo8 name.
Solo8BulletRobot = QuadrupedBulletRobot
get_solo8_robot = get_robot
