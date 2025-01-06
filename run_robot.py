import numpy as np

from src.RobotController import RobotController
from src.STservo_sdk import PortHandler, Sts
from src.XboxController import XboxController
from src.quad.Command import Command
from src.quad.Config import Configuration
from src.quad.Controller import Controller
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState
import time
import matplotlib.pyplot as plt

if __name__ == '__main__':
    config = Configuration()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    state = State()
    state.quat_orientation = np.array([1, 0, 0, 0])

    command = Command()

    last_loop = time.time()

    portHandler = PortHandler('COM5')

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

    xbox_controller = XboxController(dead_zone=30, scale=100)

    steps = 1000

    res = np.zeros((steps, 3))

    state.behavior_state = BehaviorState.INSTALL
    controller.run(state, command)
    robot.set_actuator_positions(state.joint_angles)


    # state.behavior_state = BehaviorState.REST
    # controller.run(state, command)
    # robot.set_actuator_positions(state.joint_angles)
    #
    # command.trot_event = True
    #
    # time.sleep(1)
    #
    # for step in range(steps):
    #     # now = time.time()
    #     # if now - last_loop < config.dt:
    #     #     continue
    #     # last_loop = time.time()
    #     time.sleep(config.dt)
    #
    #     command.horizontal_velocity = np.array([0.3,0])
    #
    #     controller.run(state, command)
    #
    #     command.trot_event = False
    #     robot.set_actuator_positions(state.joint_angles)
    #
    #     res[step] = state.rotated_foot_locations[:, 1]


    # ax = plt.axes(projection='3d')
    # ax.plot(res[:, 0], res[:, 1], res[:, 2], 'g')
    # ax.set_xlabel('$X$', fontsize=20)
    # ax.set_ylabel('$Y$', fontsize=20)


    # plt.show()
