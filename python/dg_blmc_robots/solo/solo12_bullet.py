from __future__ import print_function

import os
import rospkg
import numpy as np
import time

import dynamic_graph_manager as dgm
from dynamic_graph_manager.device import Device
from dynamic_graph_manager.device.robot import Robot

from robot_properties_solo.config import Solo12Config

import pybullet as p
import pinocchio as se3
from pinocchio.utils import zero

from matplotlib import pyplot as plt

from dynamic_graph.sot.core.vector_constant import VectorConstant

from py_pinocchio_bullet.wrapper import PinBulletWrapper

from dg_blmc_robots.solo.solo_base_bullet import SoloBaseRobot

class Solo12BulletRobot(SoloBaseRobot):
    def __init__(self, use_fixed_base=False, record_video=False,
                 init_sliders_pose=4*[0.5], hide_gui=False):

        super(Solo12BulletRobot, self).__init__(Solo12Config(), use_fixed_base,
                record_video, init_sliders_pose)

        if hide_gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        self.q0[0] = 0.2
        self.q0[1] = 0.0
        self.q0[2] = 0.25
        self.q0[6] = 1.
        self.q0[7] = 0.0
        self.q0[8] = 0.8
        self.q0[9] = -1.6
        self.q0[10] = 0.0
        self.q0[11] = 0.8
        self.q0[12] = -1.6
        self.q0[13] = 0.0
        self.q0[14] = -0.8
        self.q0[15] = 1.6
        self.q0[16] = 0.0
        self.q0[17] = -0.8
        self.q0[18] = 1.6

        # Sync the current robot state to the graph input signals.
        self.sim2signal_()


def get_solo12_robot(use_fixed_base=False, record_video = False,
              init_sliders_pose=4*[0.5], hide_gui=False):
    return Solo12BulletRobot(use_fixed_base, record_video, init_sliders_pose,
                hide_gui)
