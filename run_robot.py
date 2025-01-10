import numpy as np

from src.JoystickInterface import JoystickInterface
from src.RobotController import RobotController
from src.STservo_sdk import PortHandler, Sts
from src.XboxController import XboxController
from src.quad.Command import Command
from src.quad.Config import Configuration
from src.quad.Controller import Controller
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
import time

if __name__ == '__main__':
    config = Configuration()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    state = State(config)
    state.quat_orientation = np.array([1, 0, 0, 0])

    command = Command(config)

    last_loop = time.time()

    portHandler = PortHandler('COM6')

    packetHandler = Sts(portHandler)

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(1000000):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    robot = RobotController(packetHandler)

    xboxController = XboxController(scale=1, dead_zone=0.2)
    joystick_interface = JoystickInterface(config, xboxController, enable_install=True)

    delay_factor = 1.5
    # while True:
    #     print("Waiting for start to activate robot.")
    #     while True:
    #         command = joystick_interface.get_command(state, config)
    #         if command.activate_event:
    #             break
    #         time.sleep(0.1)
    #     print("Robot activated.")

    state.behavior_state = BehaviorState.REST
    while True:
        now = time.time()
        if now - last_loop < config.dt*delay_factor:
            continue
        last_loop = time.time()

        command = joystick_interface.get_command(state, config)
        controller.run(state, command)
        robot.set_actuator_positions(state.joint_angles)
