import numpy as np

from src.quad import Config


class QuadCommand:

    def __init__(self, config):
        self.horizontal_velocity = np.array([0, 0])
        self.yaw_rate = 0.0
        self.height = config.default_z_ref
        self.pitch = 0.0
        self.roll = 0.0
        self.activation = 0