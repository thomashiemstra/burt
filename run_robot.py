from unittest import TestCase

import numpy as np

from src.quad.Command import Command
from src.quad.Config import Configuration
from src.quad.Controller import Controller
from src.quad.Kinematics import four_legs_inverse_kinematics
from src.quad.State import State, BehaviorState


if __name__ == '__main__':
    config = Configuration()

    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )

    state = State()
    state.quat_orientation = np.array([1, 0, 0, 0])

    command = Command()

    steps = 1

    res = np.zeros((steps, 3))
    # command.trot_event = True
    angles = np.zeros((steps, 3))

    state.behavior_state = BehaviorState.REST

    for step in range(steps):

        command.horizontal_velocity = np.array([0.2, 0])

        controller.run(state, command)
        # command.trot_event = False
        # print(state.foot_locations)
        res[step] = state.foot_locations[:, 0]
        angles[step] = state.joint_angles[:, 0]

    # print(res)
    stuff = np.array([[1, 4, 7, 10],
                                   [2, 5, 8, 11],
                                   [3, 6, 9, 12]])
    print(angles)


    # ax = plt.axes(projection='3d')
    # ax.plot(res[:, 0], res[:, 1], res[:, 2], 'g')
    #
    # plt.show()
