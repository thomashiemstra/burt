import os

from src.STservo_sdk import Sts, GroupSyncWrite, STS_GOAL_POSITION_L, PortHandler
import numpy as np
from numpy import pi
from typing import cast

from src.Util import is_windows

STS_MAXIMUM_POSITION_VALUE = 4095
STS_MOVING_SPEED = 2400  # STServo moving speed
STS_MOVING_ACC = 50
MIN_ANGLE = -pi
MAX_ANGLE = pi
MIN_POSITION = 0
MAX_POSITION = 4096
COMM_SUCCESS = 0


class Servo:

    def __init__(self, id, min_angle, max_angle, min_pos, max_pos):
        self.id = id
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.min_pos = min_pos
        self.max_pos = max_pos
        self._offset = 0

    def offset(self, offset):
        self._offset = offset

    def angle_to_position(self, angle):
        return int(np.rint(np.interp(angle, [self.min_angle, self.max_angle], [self.min_pos, self.max_pos]))) + self._offset


class RobotController:

    def __init__(self, packet_handler: Sts, config):
        self.packet_handler = packet_handler
        self.servo_mapping = np.array([[Servo(1, -pi, pi, 4095, 0), Servo(4, -pi, pi, 4095, 0), Servo(7, -pi, pi, 4095, 0), Servo(10, -pi, pi, 4095, 0)],
                                       [Servo(2, -pi, pi, 4095, 0), Servo(5, -pi, pi, 0, 4095), Servo(8, -pi, pi, 4095, 0), Servo(11, -pi, pi, 0, 4095)],
                                       [Servo(3, -pi, pi, 0, 4095), Servo(6, -pi, pi, 4095, 0), Servo(9, -pi, pi, 0, 4095), Servo(12, -pi, pi, 4095, 0)]])

        for i, offset in enumerate(config.offsets):
            self._get_servo_by_id(i+1).offset(offset)

        self.packet_handler.groupSyncWrite = GroupSyncWrite(self.packet_handler, STS_GOAL_POSITION_L, 2)

    def _get_servo_by_id(self, servo_id):
        for leg_index in range(4):
            for axis_index in range(3):
                servo = cast(Servo, self.servo_mapping[axis_index, leg_index])
                if servo.id == servo_id:
                    return servo

    def set_actuator_positions(self, joint_angles):
        self.packet_handler.groupSyncWrite.clearParam()
        for leg_index in range(4):
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                servo = cast(Servo, self.servo_mapping[axis_index, leg_index])
                position = servo.angle_to_position(angle)

                # print("writing angle: {} to servo: {} with position: {}".format(angle, servo.id, position))

                sts_add_param_result = self.packet_handler.SyncWritePos(servo.id, position)
                if not sts_add_param_result:
                    print("[ID:%03d] groupSyncWrite add param failed" % servo.id)

        sts_comm_result = self.packet_handler.groupSyncWrite.txPacket()
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(sts_comm_result))

        # Clear sync write parameter storage
        self.packet_handler.groupSyncWrite.clearParam()


def setup_robot_controller(config):
    port = get_port(config)
    portHandler = PortHandler(port)

    packetHandler = Sts(portHandler)

    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(config.baud_rate):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    return RobotController(packetHandler, config)


def get_port(config):
    return config.windows_com_port if is_windows() else config.linux_com_port