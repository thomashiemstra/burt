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

# Default setting
STS_ID                  = 1                 # STServo ID : 1
BAUDRATE                = 1000000           # STServo default baudrate : 1000000
DEVICENAME              = 'COM5'    # Check which port is being used on your controller
                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

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
for id in range (1, 50):
    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(id)
    if sts_comm_result == COMM_SUCCESS:
        print("[ID:%03d] ping Succeeded. STServo model number : %d" % (id, sts_model_number))
        # break

        sts_present_position, sts_present_speed, sts_comm_result, sts_error = packetHandler.ReadPosSpeed(id)
        if sts_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(sts_comm_result))
        else:
            print("[ID:%03d] PresPos:%d PresSpd:%d" % (
                id, sts_present_position, sts_present_speed))
        if sts_error != 0:
            print(packetHandler.getRxPacketError(sts_error))

    if sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))

# Close port
portHandler.closePort()
