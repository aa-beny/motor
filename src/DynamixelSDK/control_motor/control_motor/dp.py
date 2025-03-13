import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from dynamixel_sdk_custom_interfaces.msg import SetPositionArray  # 引入自定義訊息

import time
from pynput import keyboard

# 控制表地址
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116

PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICE_NAME = "/dev/ttyUSB0"  # [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

DXL_ID1 = 1

class MotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()

        # 創建一個自定義訊息的發布器
        self.position_publisher = self.create_publisher(SetPositionArray, '/motor_position_array', 10)

        # 定期發布馬達位置
        self.timer = self.create_timer(0.5, self.publish_position)

    def init_dynamixel(self):
        # 開啟串口
        dxl_comm_result = self.portHandler.openPort()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error("Failed to open the port!")
            return -1
        else:
            self.get_logger().info("Succeeded to open the port.")

        # 設置波特率
        dxl_comm_result = self.portHandler.setBaudRate(BAUDRATE)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error("Failed to set the baudrate!")
            return -1
        else:
            self.get_logger().info("Succeeded to set the baudrate.")

        # 啟用扭矩並設置為位置控制模式
        self.setupDynamixel(DXL_ID1)

    def setupDynamixel(self, dxl_id):
    # 設置為位置控制模式
        dxl_comm_result = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            dxl_id,
            ADDR_OPERATING_MODE,
            3  # 3 = 位置控制模式
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to set Position Control Mode: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
        else:
            self.get_logger().info("Succeeded to set Position Control Mode.")

        # 啟用扭矩
        Torque_dxl_comm_result = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            1
        )

        if Torque_dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f"Failed to enable torque: {self.packetHandler.getTxRxResult(Torque_dxl_comm_result)}")
        else:
            self.get_logger().info("Succeeded to enable torque.")

    def publish_position(self):
        # 假設這裡的馬達位置數據是隨機或根據需要更新的
        position_array = SetPositionArray()
        position_array.position = [100, 200, 300, 400, 500, 600, 700]  # 設定7個位置值

        # 發布訊息
        self.position_publisher.publish(position_array)
        self.get_logger().info("Published positions: %s" % position_array.position)


class Controller:
    def __init__(self):
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def on_press(self, key):
        if key == keyboard.Key.esc:
            self.stop()

    def stop(self):
        print("Exiting...")
        self.listener.stop()
    
def main(args=None):
    rclpy.init(args=args)
    dynamixel_controller = MotorController()
    controller = Controller()

    try:
        while rclpy.ok():
            rclpy.spin_once(dynamixel_controller)
            time.sleep(0.1)
            if not controller.listener.running:
                break
    finally:
        # 清理
        dynamixel_controller.packetHandler.write1ByteTxRx(
            dynamixel_controller.portHandler,
            DXL_ID1,
            ADDR_TORQUE_ENABLE,
            0
        )
        dynamixel_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

