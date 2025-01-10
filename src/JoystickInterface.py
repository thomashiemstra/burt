import numpy as np

from src.XboxController import ControllerState
from src.quad.Command import Command
from src.quad.State import BehaviorState
from typing import cast


def deadband(value, band_radius):
    return max(value - band_radius, 0) + min(value + band_radius, 0)


def clipped_first_order_filter(input, target, max_rate, tau):
    rate = (target - input) / tau
    return np.clip(rate, -max_rate, max_rate)


class JoystickInterface:
    def __init__(
        self, config, controller, enable_install = False
    ):
        self.config = config
        self.previous_gait_toggle = 0
        self.previous_state = BehaviorState.REST
        self.previous_activate_toggle = 0
        self.previous_install_toggle = 0
        self.controller = controller
        self.enable_install = enable_install

    def get_command(self, state, config):
        controller_state = cast(ControllerState, self.controller.get_controller_state())

        command = Command(config)

        ####### Handle discrete commands ########
        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = controller_state.rb
        command.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        activate_toggle = controller_state.start
        command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

        install_toggle = controller_state.select
        command.install_event = (install_toggle == 1 and self.previous_install_toggle == 0) and self.enable_install

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_activate_toggle = activate_toggle
        self.previous_install_toggle = install_toggle

        ####### Handle continuous commands ########
        x_vel = np.clip(controller_state.l_thumb_y * self.config.max_x_velocity, -self.config.max_x_velocity/2, self.config.max_x_velocity)
        y_vel = controller_state.l_thumb_x * self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = controller_state.r_thumb_x * self.config.max_yaw_rate

        message_dt = 1.0 / 10

        # pitch = controller_state.r_thumb_y * self.config.max_pitch
        # deadbanded_pitch = deadband(
        #     pitch, self.config.pitch_deadband
        # )
        # pitch_rate = clipped_first_order_filter(
        #     state.pitch,
        #     deadbanded_pitch,
        #     self.config.max_pitch_rate,
        #     self.config.pitch_time_constant,
        # )
        # command.pitch = state.pitch + message_dt * pitch_rate

        height_movement = controller_state.lr_trigger
        command.height = state.height - 0.01 * self.config.z_speed * height_movement

        roll_movement = self._get_pad_x_direction(controller_state)
        command.roll = state.roll + message_dt * self.config.roll_speed * roll_movement

        return command

    @staticmethod
    def _get_pad_x_direction(controller_state):
        if controller_state.pad_right:
            return 1
        if controller_state.pad_left:
            return -1
        return 0