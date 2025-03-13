from dynamixel_sdk import *

ADDR_GOAL_VELOCITY = 104
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DEVICE_NAME = "/dev/ttyUSB0"
DXL_ID = 1

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# 設定速度
goal_velocity = 100
packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, goal_velocity)

# 結束時關閉馬達
packetHandler.write1ByteTxRx(portHandler, DXL_ID, 64, 0)
portHandler.closePort()

