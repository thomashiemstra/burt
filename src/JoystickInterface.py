import numpy as np

from src.StateCommand import StateCommand
from src.XboxController import ControllerState
from src.arm.RobotArmCommand import RobotArmCommand
from src.quad.QuadCommand import QuadCommand
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
        self.previous_robot_arm_toggle = 0
        self.previous_pad_right = 0
        self.previous_pad_left = 0
        self.controller = controller
        self.enable_install = enable_install
        self.previous_height = config.default_z_ref
        self.controller_state = None

    def get_state_command(self):
        self.controller_state = cast(ControllerState, self.controller.get_controller_state())

        command = StateCommand()

        # Check if requesting a state transition to trotting, or from trotting to resting
        gait_toggle = self.controller_state.y
        command.trot_event = (gait_toggle == 1 and self.previous_gait_toggle == 0)

        activate_toggle = self.controller_state.start
        command.activate_event = (activate_toggle == 1 and self.previous_activate_toggle == 0)

        install_toggle = self.controller_state.b
        command.install_event = (install_toggle == 1 and self.previous_install_toggle == 0) and self.enable_install

        robot_arm_toggle = self.controller_state.select
        command.robot_arm_event = (robot_arm_toggle == 1 and self.previous_robot_arm_toggle == 0)

        # Update previous values for toggles and state
        self.previous_gait_toggle = gait_toggle
        self.previous_activate_toggle = activate_toggle
        self.previous_install_toggle = install_toggle
        self.previous_robot_arm_toggle = robot_arm_toggle
        return command

    def get_quad_robot_command(self, state, config):
        self.controller_state = self._get_controller_state()

        command = QuadCommand(config)

        x_vel = np.clip(self.controller_state.l_thumb_y * self.config.max_x_velocity, -self.config.max_x_velocity/2, self.config.max_x_velocity)
        y_vel = self.controller_state.l_thumb_x * self.config.max_y_velocity
        command.horizontal_velocity = np.array([x_vel, y_vel])
        command.yaw_rate = self.controller_state.r_thumb_x * self.config.max_yaw_rate

        pad_right = self.controller_state.pad_right
        if pad_right and self.previous_pad_right == 0:
            state.stance = state.stance.next()

        pad_left = self.controller_state.pad_left
        if pad_left and self.previous_pad_left == 0:
            state.stance = state.stance.previous()

        self.previous_pad_right = pad_right
        self.previous_pad_left = pad_left

        height_movement = self.controller_state.lr_trigger
        command.height = state.height - 0.01 * self.config.z_speed * height_movement
        if command.height != self.previous_height:
            print("new height:" + str(command.height))
        self.previous_height = command.height

        return command

    def get_robot_arm_command(self, config):
        self.controller_state = self._get_controller_state()

        command = RobotArmCommand(config)

        height_movement = self.controller_state.lr_trigger
        command.z = command.z - 0.01 * self.config.z_speed * height_movement

        return command

    def _get_controller_state(self):
        return self.controller_state if self.controller_state is not None else cast(ControllerState, self.controller.get_controller_state())

    @staticmethod
    def _get_pad_x_direction(controller_state):
        if controller_state.pad_right:
            return 1
        if controller_state.pad_left:
            return -1
        return 0

    @staticmethod
    def _get_pad_y_direction(controller_state):
        if controller_state.pad_up:
            return 1
        if controller_state.pad_down:
            return -1
        return 0

    @staticmethod
    def _get_buttons_y_direction(controller_state):
        if controller_state.a:
            return 1
        if controller_state.y:
            return -1
        return 0

    @staticmethod
    def _get_buttons_x_direction(controller_state):
        if controller_state.b:
            return 1
        if controller_state.x:
            return -1
        return 0

    @staticmethod
    def _get_bumper_direction(controller_state):
        if controller_state.rb:
            return 1
        if controller_state.lb:
            return -1
        return 0
