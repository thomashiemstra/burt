#!/usr/bin/env python
#
# *********     Ping Example      *********
#
#
# Available STServo model on this example : All models using Protocol STS
# This example is tested with a STServo and an URT
#

import sys

sys.path.append("../..")
from src.STservo_sdk import *                   # Uses STServo SDK library
import numpy as np

# Default setting
STS_ID                  = 1                 # STServo ID : 1
BAUDRATE                = 1000000           # STServo default baudrate : 1000000
DEVICENAME              = 'COM6'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]


positions = [1024, 2048, 3072]

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = Sts(portHandler)
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    quit()

# Try to ping the STServo
# Get STServo model number
offsets = []
for id in range (1, 20):
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(id)
    if sts_comm_result == COMM_SUCCESS:
        # print("[ID:%03d] ping Succeeded. STServo model number : %d" % (id, sts_model_number))
        # break

        sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(id)

        goal_pos = find_nearest(positions, sts_present_position)

        offset = sts_present_position - goal_pos

        offsets.append(int(offset))

        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print("[ID:%03d] Pos:%d goalPos:%d offset:%d" % (
                id, sts_present_position, goal_pos, offset))
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))

    if sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))

# Close port
print(offsets)
portHandler.closePort()
