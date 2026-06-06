"""Run the *simulated* robot.

Mirrors run_robot.py, but instead of talking to the physical STServo bus it
loads urdf/burt.urdf onto a floor in PyBullet and drives it with the same
QuadController + IK + joystick stack. The physical robot has a robot arm; the
simulated one does not, so all arm behaviour is omitted here.

Controls (Xbox controller, identical to the real robot):
    A            activate / deactivate (enable / disable motors)
    Y            toggle trot / rest
    left stick   walk (forward/back, strafe)
    right stick  yaw
    triggers     body height
    d-pad L/R    cycle stance (slow / medium)
"""
import argparse
import os
import time

from numpy import loadtxt

from src.ConfigEditor import setup_config_editor
from src.JoystickInterface import JoystickInterface
from src.StanceManager import StanceManager
from src.XboxController import XboxController
from src.quad.Config import Configuration
from src.quad.QuadController import QuadController
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
from src.sim.SimRobotController import SimRobotController
from src.state_controller import StateController
from src.Util import is_windows

class _NoArm:
    """Stand-in for the arm controller / arm robot used by StateController.

    The simulation has no arm, so every arm transition is a no-op. enable_arm is
    False on the JoystickInterface, so the ARM behaviour state is never entered.
    """

    def run_position(self, *args, **kwargs):
        return None

    def activate_arm_if_not_already(self, *args, **kwargs):
        pass

    def deactivate_arm_if_not_already(self, *args, **kwargs):
        pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run the simulated (URDF) robot in PyBullet.")
    parser.add_argument("--use-editor", action="store_true",
                        help="Open the live config editor window.")
    args = parser.parse_args()

    script_dir = os.path.dirname(__file__)
    servo_offsets = loadtxt(
        os.path.join(script_dir, "src/quad/offsets.txt"),
        comments="#", delimiter=",", unpack=False, dtype=int,
    )

    config = Configuration(servo_offsets)

    quad_controller = QuadController(config, four_legs_inverse_kinematics)
    quad_robot = SimRobotController(config, os.path.join(script_dir, "urdf/burt.urdf"), gui=True)

    root = None
    if args.use_editor:
        root = setup_config_editor(config)

    no_arm = _NoArm()
    xbox_controller = XboxController(scale=1, dead_zone=0.2)
    joystick_interface = JoystickInterface(config, xbox_controller, enable_arm=False,
                                           enable_install=is_windows())
    stance_manager = StanceManager()
    state_controller = StateController(no_arm, config, no_arm, quad_robot)

    state = State(config)
    # Unlike the real robot (which boots with motors off for safety), start the
    # sim powered on and standing in REST so it holds the stance instead of going
    # limp and collapsing under gravity. Press A to toggle motors off (it will sit
    # down), Y to trot.
    state.behavior_state = BehaviorState.REST
    previous_stance = state.stance
    stance_manager.apply_stance(state.stance, config)
    quad_robot.enable_motors()

    # Step physics fast enough for stability, but only recompute the gait at the
    # same control cadence the real robot uses (config.dt * delay_factor).
    physics_dt = 1.0 / 240.0
    control_period = config.dt * config.delay_factor
    substeps = max(1, round(control_period / physics_dt))

    previous_start = False

    while True:
        loop_start = time.time()

        state_command = joystick_interface.get_state_command()
        state_controller.run(state, None, state_command)
        state_controller.handle_state_change(state_command, state, None)

        # Start button (rising edge) teleports the robot back to its spawn pose.
        start_pressed = joystick_interface.controller_state.start
        if start_pressed and not previous_start:
            quad_robot.reset()
        previous_start = start_pressed

        if state.stance != previous_stance:
            stance_manager.apply_stance(state.stance, config)
            previous_stance = state.stance

        quad_command = joystick_interface.get_quad_robot_command(state, config)
        # End to end (this controller + gait) the lateral channels are mirrored:
        # right stick right turns the body left, and left stick right strafes
        # left. Negate both so the sim steers/strafes the intuitive way (forward
        # back is already correct). The real robot (run_robot.py) uses the same
        # controller + gait and is not touched by these lines - to fix both,
        # negate yaw_rate and horizontal_velocity[1] in
        # JoystickInterface.get_quad_robot_command instead.
        quad_command.yaw_rate = -quad_command.yaw_rate
        quad_command.horizontal_velocity[1] = -quad_command.horizontal_velocity[1]
        quad_controller.run(state, quad_command)
        quad_robot.set_actuator_positions(state)

        quad_robot.step(substeps)

        elapsed = time.time() - loop_start
        if elapsed < control_period:
            time.sleep(control_period - elapsed)

        if root is not None:
            root.update()
