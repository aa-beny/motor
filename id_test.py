from dynamixel_sdk import *

ADDR_MODEL_NUMBER = 0
BAUDRATE = 115200  # 如果沒找到，試試 1000000
PROTOCOL_VERSION = 2.0
DEVICE_NAME = "/dev/ttyUSB0"  # 確保是正確的 USB 端口

portHandler = PortHandler(DEVICE_NAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print("❌ 無法開啟串口！")
    exit()

if not portHandler.setBaudRate(BAUDRATE):
    print("❌ 設定波特率失敗！")
    exit()

print("🔍 正在掃描 Dynamixel ID...")
for dxl_id in range(1, 10):  # 一般 ID 介於 1-10
    dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
    if dxl_comm_result == COMM_SUCCESS:
        print(f"✅ 找到馬達！ID: {dxl_id}，型號: {dxl_model_number}")
    else:
        print(f"❌ ID {dxl_id} 沒有回應")

portHandler.closePort()

