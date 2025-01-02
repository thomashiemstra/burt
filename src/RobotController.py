from src.STservo_sdk import Sts
import numpy as np
from numpy import pi

STS_MAXIMUM_POSITION_VALUE = 4095
STS_MOVING_SPEED = 2400  # STServo moving speed
STS_MOVING_ACC = 50
MIN_ANGLE = -pi
MAX_ANGLE = pi
MIN_POSITION = 0
MAX_POSITION = 4096
COMM_SUCCESS = 0


def angle_to_position(angle):
    return int(np.rint(np.interp(angle, [MIN_ANGLE, MAX_ANGLE], [MIN_POSITION, MAX_POSITION])))


class RobotController:

    def __init__(self, packet_handler: Sts):
        self.packet_handler = packet_handler
        self.servo_mapping = np.array([[1, 4, 7, 10],
                                       [2, 5, 8, 11],
                                       [3, 6, 9, 12]])

    def set_actuator_positions(self, joint_angles):
        for leg_index in range(4):
            for axis_index in range(3):
                angle = joint_angles[axis_index, leg_index]
                servo_id = int(self.servo_mapping[axis_index, leg_index])

                sts_add_param_result = self.packet_handler.SyncWritePosEx(servo_id, angle, STS_MOVING_SPEED,
                                                                          STS_MOVING_ACC)
                if not sts_add_param_result:
                    print("[ID:%03d] groupSyncWrite add param failed" % servo_id)

        sts_comm_result = self.packet_handler.groupSyncWrite.txPacket()
        if sts_comm_result != COMM_SUCCESS:
            print("%s" % self.packet_handler.getTxRxResult(sts_comm_result))

        # Clear sync write parameter storage
        self.packet_handler.groupSyncWrite.clearParam()