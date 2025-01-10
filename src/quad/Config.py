import numpy as np
from enum import Enum


class Configuration:
    def __init__(self):
                #################### COMMANDS ####################
        self.max_x_velocity = 0.2
        self.max_y_velocity = 0.1
        self.max_yaw_rate = 1.0
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
        self.dt = 0.005
        self.num_phases = 4
        self.contact_phases = np.array(
            [[1, 1, 1, 0], [1, 0, 1, 1], [1, 0, 1, 1], [1, 1, 1, 0]]
        )
        self.overlap_time = (
            0.08 # duration of the phase where all four feet are on the ground
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
        self.delta_x = 0.10
        self.x_shift = 0.02
        self.delta_y = 0.00
        self.default_z_ref = -0.16

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

        
class SimulationConfig:
    def __init__(self):
        self.XML_IN = "pupper.xml"
        self.XML_OUT = "pupper_out.xml"

        self.START_HEIGHT = 0.3
        self.MU = 1.5  # coeff friction
        self.DT = 0.001  # seconds between simulation steps
        self.JOINT_SOLREF = "0.001 1"  # time constant and damping ratio for joints
        self.JOINT_SOLIMP = "0.9 0.95 0.001"  # joint constraint parameters
        self.GEOM_SOLREF = "0.01 1"  # time constant and damping ratio for geom contacts
        self.GEOM_SOLIMP = "0.9 0.95 0.001"  # geometry contact parameters
        
        # Joint params
        G = 220  # Servo gear ratio
        m_rotor = 0.016  # Servo rotor mass
        r_rotor = 0.005  # Rotor radius
        self.ARMATURE = G ** 2 * m_rotor * r_rotor ** 2  # Inertia of rotational joints
        # print("Servo armature", self.ARMATURE)

        NATURAL_DAMPING = 1.0  # Damping resulting from friction
        ELECTRICAL_DAMPING = 0.049  # Damping resulting from back-EMF

        self.REV_DAMPING = (
            NATURAL_DAMPING + ELECTRICAL_DAMPING
        )  # Damping torque on the revolute joints

        # Servo params
        self.SERVO_REV_KP = 300  # Position gain [Nm/rad]

        # Force limits
        self.MAX_JOINT_TORQUE = 3.0
        self.REVOLUTE_RANGE = 1.57
