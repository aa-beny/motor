from dynamixel_sdk import *

ADDR_GOAL_VELOCITY = 104
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
DEVICE_NAME = "/dev/ttyUSB0"  # 確保這是正確的裝置
DXL_ID = 1

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# 嘗試 Ping Dynamixel
dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, DXL_ID)

if dxl_comm_result != COMM_SUCCESS:
    print(f"Ping 失敗: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Ping 成功，但有錯誤: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"Ping 成功！馬達型號: {dxl_model_number}")

portHandler.closePort()

