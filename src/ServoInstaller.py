from time import sleep

from src.STservo_sdk import PortHandler, Sts

BAUDRATE = 1000000  # STServo default baudrate : 1000000
DEVICENAME = 'COM5'
P = 50
D = 32
I = 0
ACCELERATION = 254

NEW_ID = 1
STS_MOVING_SPEED            = 0        # STServo moving speed
STS_MOVING_ACC              = 0         # STServo moving acc

portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = Sts(portHandler)


def handle_result(com_res, error):
    if sts_comm_result != 0:
        print("%s" % packetHandler.getTxRxResult(com_res))
    elif sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(error))


def get_servo_id():
    for id in range(1, 254):
        sts_model_number, sts_comm_result, sts_error = packetHandler.ping(id)
        if sts_comm_result == 0:
            print("[ID:%03d] ping Succeeded. STServo model number : %d" % (id, sts_model_number))
            return id
        if sts_error != 0:
            print("%s" % packetHandler.getRxPacketError(sts_error))


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

old_id = get_servo_id()

packetHandler.unLockEprom(old_id)

if NEW_ID != old_id:
    print("writing new id")
    sts_comm_result, sts_error = packetHandler.setId(old_id, NEW_ID)
    handle_result(sts_comm_result, sts_error)

    sts_model_number, sts_comm_result, sts_error = packetHandler.ping(NEW_ID)
    if sts_error != 0:
        print("%s" % packetHandler.getRxPacketError(sts_error))
        exit()
    if sts_comm_result == 0:
        print("[ID:%03d] ping Succeeded. STServo model number : %d" % (NEW_ID, sts_model_number))
    else:
        print("writing new id failed")
        exit()


print("writing  P:")
sts_comm_result, sts_error = packetHandler.setP(NEW_ID, P)
handle_result(sts_comm_result, sts_error)

print("writing  I:")
sts_comm_result, sts_error = packetHandler.setI(NEW_ID, I)
handle_result(sts_comm_result, sts_error)

print("writing  D:")
sts_comm_result, sts_error = packetHandler.setD(NEW_ID, D)
handle_result(sts_comm_result, sts_error)

# print("writing pos 1")
# sts_comm_result, sts_error = packetHandler.WritePos(NEW_ID, 0)
# handle_result(sts_comm_result, sts_error)
#
# sleep(2)
#
# print("writing pos 2")
# sts_comm_result, sts_error = packetHandler.WritePos(NEW_ID, 4095)
# handle_result(sts_comm_result, sts_error)
#
# sleep(2)
#
# print("writing neutral pos")
# sts_comm_result, sts_error = packetHandler.WritePos(NEW_ID, 2048)
# handle_result(sts_comm_result, sts_error)

# Close port
packetHandler.LockEprom(NEW_ID)
portHandler.closePort()