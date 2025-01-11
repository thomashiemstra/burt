import time

from src.ConfigEditor import setup_editor
from src.JoystickInterface import JoystickInterface
from src.RobotController import setup_robot_controller
from src.XboxController import XboxController
from src.quad.Config import Configuration
from src.quad.Controller import Controller
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State

if __name__ == '__main__':
    config = Configuration()

    controller = Controller(config, four_legs_inverse_kinematics)
    robot = setup_robot_controller(config)
    xboxController = XboxController(scale=1, dead_zone=0.2)
    joystick_interface = JoystickInterface(config, xboxController, enable_install=True)

    root = setup_editor(config)

    state = State(config)
    # while True:
    #     print("Waiting for start to activate robot.")
    #     while True:
    #         command = joystick_interface.get_command(state, config)
    #         if command.activate_event:
    #             break
    #         time.sleep(0.1)
    #     print("Robot activated.")

    last_loop = time.time()

    while True:
        now = time.time()
        if now - last_loop < config.dt * config.delay_factor:
            continue
        last_loop = time.time()

        command = joystick_interface.get_command(state, config)
        controller.run(state, command)
        robot.set_actuator_positions(state.joint_angles)
        root.update()
