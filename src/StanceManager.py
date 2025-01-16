from enum import Enum
import numpy as np


class StanceConfig:

    def __init__(self,
                 max_x_velocity, max_y_velocity, max_yaw_rate, swing_time, overlap_time, delay_factor, alpha, beta, z_clearance, delta_x, x_shift, delta_y, ):
        self.max_yaw_rate = max_yaw_rate
        self.max_y_velocity = max_y_velocity
        self.max_x_velocity = max_x_velocity
        self.swing_time = swing_time
        self.overlap_time = overlap_time
        self.delay_factor = delay_factor
        self.alpha = alpha
        self.beta = beta
        self.z_clearance = z_clearance
        self.delta_x = delta_x
        self.x_shift = x_shift
        self.delta_y = delta_y

    def apply_stance_to_config(self, config):
        config.swing_time = self.swing_time
        config.overlap_time = self.overlap_time
        config.delay_factor = self.delay_factor
        config.alpha = self.alpha
        config.beta = self.beta
        config.z_clearance = self.z_clearance
        config.delta_x = self.delta_x
        config.x_shift = self.x_shift
        config.delta_y = self.delta_y
        config.max_yaw_rate = self.max_yaw_rate
        config.max_y_velocity = self.max_y_velocity
        config.max_x_velocity = self.max_x_velocity


class StanceManager(object):

    def __init__(self):
        self.configs = {
            Stance.SLOW:   StanceConfig(max_x_velocity=0.2, max_y_velocity=0.1, max_yaw_rate=0.5, swing_time=0.1, overlap_time=0.1, delay_factor=1.24, alpha=0.5, beta=0.5, z_clearance=0.67, delta_x=0.09, x_shift=0.02, delta_y=0.0),
            Stance.MEDIUM: StanceConfig(max_x_velocity=0.2, max_y_velocity=0.1, max_yaw_rate=0.5, swing_time=0.1, overlap_time=0.1, delay_factor=1.24, alpha=0.5, beta=0.5, z_clearance=0.67, delta_x=0.12, x_shift=0.02, delta_y=0.01),
            Stance.FAST:   StanceConfig(max_x_velocity=0.2, max_y_velocity=0.1, max_yaw_rate=0.5, swing_time=0.1, overlap_time=0.1, delay_factor=1.24, alpha=0.5, beta=0.5, z_clearance=0.67, delta_x=0.15, x_shift=0.02, delta_y=0.03)
        }

    def apply_stance(self, stance, config):
        self.configs[stance].apply_stance_to_config(config)


class Stance(Enum):
    SLOW = 1
    MEDIUM = 2
    FAST = 3

    def next(self):
        val = np.clip(self.value + 1, 1, 3)
        return Stance(val)

    def previous(self):
        val = np.clip(self.value - 1, 1, 3)
        return Stance(val)
