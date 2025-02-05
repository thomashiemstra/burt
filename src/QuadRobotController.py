import os
from enum import Enum

from src.STservo_sdk import Sts, GroupSyncWrite, STS_GOAL_POSITION_L, PortHandler, Scscl
import numpy as np
from numpy import pi
from typing import cast
from time import sleep

from src.Util import is_windows
from src.quad.State import BehaviorState

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
        self.offset = 0

    def angle_to_position(self, angle):
        return int(np.rint(np.interp(angle, [self.min_angle, self.max_angle], [self.min_pos, self.max_pos]))) + self.offset


class QuadRobotController:

    def __init__(self, packet_handler: Sts, config):
        self.packet_handler = packet_handler
        self.servo_mapping = np.array([[Servo(1, -pi, pi, 4095, 0), Servo(4, -pi, pi, 4095, 0), Servo(7, -pi, pi, 4095, 0), Servo(10, -pi, pi, 4095, 0)],
                                       [Servo(2, -pi, pi, 4095, 0), Servo(5, -pi, pi, 0, 4095), Servo(8, -pi, pi, 4095, 0), Servo(11, -pi, pi, 0, 4095)],
                                       [Servo(3, -pi, pi, 0, 4095), Servo(6, -pi, pi, 4095, 0), Servo(9, -pi, pi, 0, 4095), Servo(12, -pi, pi, 4095, 0)]])

        for i, offset in enumerate(config.offsets):
            self._get_servo_by_id(i+1).offset = offset

        self.packet_handler.groupSyncWrite = GroupSyncWrite(self.packet_handler, STS_GOAL_POSITION_L, 2)

    def _get_servo_by_id(self, servo_id):
        for leg_index in range(4):
            for axis_index in range(3):
                servo = cast(Servo, self.servo_mapping[axis_index, leg_index])
                if servo.id == servo_id:
                    return servo

    def get_servo_list(self):
        res = []
        for i in range(1, 13):
            res.append(self._get_servo_by_id(i))
        return res

    def set_actuator_positions(self, state):
        if state.behavior_state == BehaviorState.DEACTIVATED:
            return
        self.packet_handler.groupSyncWrite.clearParam()
        for leg_index in range(4):
            for axis_index in range(3):
                angle = state.joint_angles[axis_index, leg_index]
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

    def enable_motors(self):
        for leg_index in range(4):
            for axis_index in range(3):
                servo = cast(Servo, self.servo_mapping[axis_index, leg_index])
                res = self.packet_handler.enable_torque(servo.id)
                if not res:
                    print("[ID:%03d] enable torque failed" % servo.id)

    def disable_motors(self):
        for leg_index in range(4):
            for axis_index in range(3):
                servo = cast(Servo, self.servo_mapping[axis_index, leg_index])
                res = self.packet_handler.disable_torque(servo.id)
                if not res:
                    print("[ID:%03d] disable torque failed" % servo.id)


class RobotArmRobotController:

    def __init__(self, sts_packet_handler, scscl_packet_handler, config):
        self.sts_packet_handler = sts_packet_handler
        self.scscl_packet_handler = scscl_packet_handler
        self.config = config
        self.arm_servos = np.array([Servo(13, -pi, pi, 0, 4095), Servo(14, -pi, pi, 4095, 0), Servo(15, -pi, pi, 4095, 0)])
        self.wrist_servo = Servo(16, pi/2, -pi/2, 0, 1023)
        self.state = ArmState.DEACTIVATED

    def deactivate_arm_if_not_already(self, angles):
        if self.state == ArmState.ACTIVATED:
            self.state = ArmState.DEACTIVATED
            self.disable_motors()
            self.set_arm_servos(angles, speed=self.config.activate_arm_speed)
            sleep(self.config.activate_arm_time)

    def activate_arm_if_not_already(self, angles):
        if self.state == ArmState.DEACTIVATED:
            self.state = ArmState.ACTIVATED
            self.enable_motors()
            self.set_arm_servos(angles, speed=self.config.activate_arm_speed)
            sleep(self.config.activate_arm_time)

    def set_actuator_positions(self, joint_angles):
        if self.state == ArmState.ACTIVATED:
            self.set_arm_servos(joint_angles)
        else:
            print("error, arm not activated")
            quit()

    def set_arm_servos(self, joint_angles, speed=0):
        for i, servo in enumerate(self.arm_servos):
            servo = cast(Servo, servo)
            position = servo.angle_to_position(joint_angles[i])
            sts_add_param_result = self.sts_packet_handler.SyncWritePosEx(servo.id, position, speed, 0)
            if not sts_add_param_result:
                print("[ID:%03d] groupSyncWrite add param failed" % servo.id)
        sts_comm_result = self.sts_packet_handler.groupSyncWrite.txPacket()
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.sts_packet_handler.getTxRxResult(sts_comm_result))
        # Clear sync write parameter storage
        self.sts_packet_handler.groupSyncWrite.clearParam()

        wrist_position = self.wrist_servo.angle_to_position(joint_angles[3])
        scscl_comm_result, error = self.scscl_packet_handler.WritePos(self.wrist_servo.id, wrist_position, speed)
        if scscl_comm_result != COMM_SUCCESS:
            print("%s" % self.sts_packet_handler.getTxRxResult(sts_comm_result))

    def enable_motors(self):
        for servo in self.arm_servos:
            res = self.sts_packet_handler.enable_torque(servo.id)
            if not res:
                print("[ID:%03d] enable torque failed" % servo.id)
        res = self.sts_packet_handler.enable_torque(self.wrist_servo.id)
        if not res:
            print("[ID:%03d] enable torque failed" % self.wrist_servo.id)

    def disable_motors(self):
        for servo in self.arm_servos:
            res = self.sts_packet_handler.disable_torque(servo.id)
            if not res:
                print("[ID:%03d] disable torque failed" % servo.id)
        res = self.sts_packet_handler.disable_torque(self.wrist_servo.id)
        if not res:
            print("[ID:%03d] disable torque failed" % self.wrist_servo.id)

class ArmState(Enum):
    DEACTIVATED = 0
    ACTIVATED = 1

def setup_robots(config):
    port = get_port(config)
    port_handler = PortHandler(port)

    sts_packet_handler_quad = Sts(port_handler)
    sts_packet_handler_arm = Sts(port_handler)
    scscl_packet_handler = Scscl(port_handler)

    if port_handler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        quit()

    # Set port baudrate
    if port_handler.setBaudRate(config.baud_rate):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        quit()

    return QuadRobotController(sts_packet_handler_quad, config), RobotArmRobotController(sts_packet_handler_arm, scscl_packet_handler, config)


def get_port(config):
    return config.windows_com_port if is_windows() else config.linux_com_port