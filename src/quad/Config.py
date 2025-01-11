import numpy as np

from src.Util import auto_str_newline


@auto_str_newline
class Configuration:
    def __init__(self):
        #################### CONNECTION ####################
        self.com_port = 'COM6'
        self.baud_rate = 1000000

        #################### COMMANDS ####################
        self.max_x_velocity = 0.14
        self.max_y_velocity = 0.1
        self.max_yaw_rate = 0.8
        self.max_pitch = 30.0 * np.pi / 180.0
        
        #################### MOVEMENT PARAMS ####################
        self.z_time_constant = 0.02
        self.z_speed = 0.03  # maximum speed [m/s]
        self.pitch_deadband = 0.02
        self.pitch_time_constant = 0.25
        self.max_pitch_rate = 0.15
        self.roll_speed = 0.16  # maximum roll rate [rad/s]
        self.yaw_time_constant = 0.3
        self.max_stance_yaw = 1.2
        self.max_stance_yaw_rate = 2.0

        #################### SWING ######################
        self.z_clearance = 0.1
        self.alpha = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )
        self.beta = (
            0.5  # Ratio between touchdown distance and total horizontal stance movement
        )

        #################### GAIT #######################
        self.delay_factor = 1.3
        self.dt = 0.005
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.12 # duration of the phase where all four feet are on the ground
        )
        self.swing_time = (
            0.05 # duration of the phase when only two feet are on the ground
        )

        ######################## GEOMETRY ######################
        self.LEG_FB = 0.09  # front-back distance from center line to leg axis
        self.LEG_LR = 0.06 # left-right distance from center line to leg plane
        self.LEG_L2 = 0.074
        self.LEG_L1 = 0.10
        self.FOOT_RADIUS = 0.026
        self.ABDUCTION_OFFSET = 0.03  # distance from abduction axis to leg

        self.LEG_ORIGINS = np.array(
            [
                [self.LEG_FB, self.LEG_FB, -self.LEG_FB, -self.LEG_FB],
                [-self.LEG_LR, self.LEG_LR, -self.LEG_LR, self.LEG_LR],
                [0, 0, 0, 0],
            ]
        )

        self.ABDUCTION_OFFSETS = np.array(
            [
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
                -self.ABDUCTION_OFFSET,
                self.ABDUCTION_OFFSET,
            ]
        )

        #################### STANCE ####################
        self.delta_x = 0.12
        self.x_shift = 0.02
        self.delta_y = 0.00
        self.default_z_ref = -0.15

    @property
    def default_stance(self):
        y_offset = self.LEG_LR + self.ABDUCTION_OFFSET + self.delta_y
        return np.array(
            [
                [self.delta_x + self.x_shift,  self.delta_x + self.x_shift, -self.delta_x + self.x_shift, -self.delta_x + self.x_shift],
                [-y_offset, y_offset, -y_offset,  y_offset],
                [0, 0, 0, 0],
            ]
        )

    @property
    def install_stance(self):
        z_height = -(self.LEG_L2 + self.FOOT_RADIUS)
        y_offset = self.LEG_LR + self.ABDUCTION_OFFSET
        return np.array(
            [
                [self.LEG_FB - self.LEG_L1, self.LEG_FB - self.LEG_L1, -(self.LEG_FB + self.LEG_L1), -(self.LEG_FB + self.LEG_L1)],
                [-y_offset, y_offset, -y_offset, y_offset],
                [z_height, z_height, z_height, z_height],
            ]
        )

    ################## SWING ###########################
    @property
    def z_clearance(self):
        return self.__z_clearance

    @z_clearance.setter
    def z_clearance(self, z):
        self.__z_clearance = z

    ########################### GAIT ####################
    @property
    def overlap_ticks(self):
        return int(self.overlap_time / self.dt)

    @property
    def swing_ticks(self):
        return int(self.swing_time / self.dt)

    @property
    def stance_ticks(self):
        return 2 * self.overlap_ticks + self.swing_ticks

    @property
    def phase_ticks(self):
        return np.array(
            [self.overlap_ticks, self.swing_ticks, self.overlap_ticks, self.swing_ticks]
        )

    @property
    def phase_length(self):
        return 2 * self.overlap_ticks + 2 * self.swing_ticks

    def __str__(self):
        return "wew"


