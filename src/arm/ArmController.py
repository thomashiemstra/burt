import numpy as np
from numpy import arctan2, power, sqrt, cos, sin


class ArmController:

    def __init__(self, config):
        self.config = config

    def run_command(self, command):
        return self.run(command.x, command.y, command.z, command.phi)

    def run_position(self, pos):
        return self.run(pos[0], pos[1], pos[2], pos[3])

    def run(self, x, y, z, phi):

        # First find the position of the wrist
        # TODO sin cos stuff here
        xc = x - self.config.d6 * cos(phi)
        yc = y
        zc = z - self.config.d6 * sin(phi)

        angles = np.zeros(4, dtype=np.float64)

        # The first 3 angles only depend on the position of the wrist
        angles[0] = arctan2(yc, xc)

        d1 = self.config.d1
        a2 = self.config.a2
        d4 = self.config.d4

        d = (power(xc, 2) + power(yc, 2) + power((zc - d1), 2) - power(a2, 2) - power(d4, 2)) / (2.0 * a2 * d4)
        if d >= 1 or d <= -1:
            d = 1
        angles[2] = arctan2(-sqrt(1 - d ** 2), d)

        k1 = a2 + d4 * cos(angles[3])
        k2 = d4 * sin(angles[3])
        # The positive square root is picked meaning elbow up.
        angles[1] = arctan2((zc - d1), sqrt(power(xc, 2) + power(yc, 2))) - arctan2(k2, k1)

        angles[3] = phi
        return angles
