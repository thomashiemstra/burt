import time

from src.ConfigEditor import setup_config_editor, setup_servo_editor
from src.JoystickInterface import JoystickInterface
from src.QuadRobotController import setup_robots
from src.StanceManager import StanceManager
from src.XboxController import XboxController
from src.arm.ArmController import ArmController
from src.quad.Config import Configuration
from src.quad.QuadController import QuadController
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
from src.state_controller import StateController

if __name__ == '__main__':
    config = Configuration()

    quad_controller = QuadController(config, four_legs_inverse_kinematics)
    arm_controller = ArmController(config)

    quad_robot, robot_arm = setup_robots(config)
    xboxController = XboxController(scale=1, dead_zone=0.2)
    joystick_interface = JoystickInterface(config, xboxController, enable_install=True)
    stance_manager = StanceManager()
    state_controller = StateController(arm_controller, config, robot_arm, quad_robot)

    root = None
    # root = setup_config_editor(config)
    # servos = robot.get_servo_list()
    # root = setup_servo_editor(servos)

    state = State(config)
    previous_stance = state.stance
    stance_manager.apply_stance(state.stance, config)

    last_loop = time.time()

    while True:
        now = time.time()
        if now - last_loop < config.dt * config.delay_factor:
            continue
        last_loop = time.time()

        if state.stance != previous_stance:
            stance_manager.apply_stance(state.stance, config)
            previous_stance = state.stance

        state_command = joystick_interface.get_state_command()
        state_controller.run(state, state_command)

        if state.behavior_state == BehaviorState.ARM:
            robot_arm_command = joystick_interface.get_robot_arm_command(config)
            arm_angles = arm_controller.run_command(robot_arm_command)
            robot_arm.set_actuator_positions(arm_angles)
        else:
            quad_command = joystick_interface.get_quad_robot_command(state, config)
            quad_controller.run(state, quad_command)
            quad_robot.set_actuator_positions(state)

        # root.update()
