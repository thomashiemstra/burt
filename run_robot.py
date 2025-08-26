import time

from src.ConfigEditor import setup_config_editor, setup_servo_editor
from src.JoystickInterface import JoystickInterface
from src.QuadRobotController import setup_robots
from src.StanceManager import StanceManager
from src.XboxController import XboxController
from src.arm.ArmController import ArmController
from src.arm.RobotArmState import RobotArmState
from src.quad.Config import Configuration
from src.quad.QuadController import QuadController
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
from src.state_controller import StateController
from numpy import loadtxt
from src.Util import is_windows
import os


if __name__ == '__main__':
    script_dir = os.path.dirname(__file__)
    rel_path = "src/quad/offsets.txt"
    abs_file_path = os.path.join(script_dir, rel_path)
    servo_offsets = loadtxt(abs_file_path, comments="#", delimiter=",", unpack=False, dtype=int)

    config = Configuration(servo_offsets)

    quad_controller = QuadController(config, four_legs_inverse_kinematics)
    arm_controller = ArmController(config)

    quad_robot, robot_arm = setup_robots(config)
    xboxController = XboxController(scale=1, dead_zone=0.2)
    joystick_interface = JoystickInterface(config, xboxController, enable_arm=True, enable_install=is_windows())
    stance_manager = StanceManager()
    state_controller = StateController(arm_controller, config, robot_arm, quad_robot)

    # root = setup_config_editor(config)
    # servos = quad_robot.get_servo_list()
    # root = setup_servo_editor(servos)

    state = State(config)
    arm_state = RobotArmState(config)
    previous_stance = state.stance
    stance_manager.apply_stance(state.stance, config)
    quad_robot.disable_motors()

    last_loop = time.time()

    while True:
        now = time.time()
        if now - last_loop < config.dt * config.delay_factor:
            continue
        last_loop = time.time()

        state_command = joystick_interface.get_state_command()
        state_controller.run(state, arm_state, state_command)
        state_controller.handle_state_change(state_command, state, arm_state)

        if state.stance != previous_stance:
            stance_manager.apply_stance(state.stance, config)
            previous_stance = state.stance

        if state.behavior_state == BehaviorState.ARM:
            joystick_interface.update_arm_state(config, arm_state)
            arm_angles = arm_controller.run_command(arm_state)
            robot_arm.set_actuator_positions(arm_angles)
        else:
            quad_command = joystick_interface.get_quad_robot_command(state, config)
            quad_controller.run(state, quad_command)
            quad_robot.set_actuator_positions(state)

        # if root is not None:
        #     root.update()
