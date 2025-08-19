import numpy as np
from enum import Enum

from src.StanceManager import Stance


class State:
    def __init__(self, config):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0.0
        self.height = config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0

        self.ticks = 0
        self.foot_locations = np.zeros((3, 4))
        self.rotated_foot_locations = np.zeros((3, 4))
        self.joint_angles = np.zeros((3, 4))

        self.behavior_state = BehaviorState.DEACTIVATED
        self.quat_orientation = np.array([1, 0, 0, 0])
        self.stance = Stance.MEDIUM


class BehaviorState(Enum):
    DEACTIVATED = -1
    REST = 0
    TROT = 1
    ARM = 2
    INSTALL = 3
