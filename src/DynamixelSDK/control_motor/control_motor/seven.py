import rclpy
from rclpy.node import Node
from dynamixel_sdk import *                    
from dynamixel_sdk_custom_interfaces.msg import SetPositionArray

import time
from pynput import keyboard

# Control table address
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116

# Protocol version
PROTOCOL_VERSION = 2.0
BAUDRATE = 57600
DEVICE_NAME = "/dev/ttyUSB0"

# Define motor IDs (1~7)
DXL_IDS = [1, 2, 3, 4, 5, 6, 7]

class MotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()

        self.position_subscriber = self.create_subscription(
            SetPositionArray, '/motor_position_array', self.position_callback, 10)

    def init_dynamixel(self):
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the port!")
            return
        self.get_logger().info("Succeeded to open the port.")

        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set the baudrate!")
            return
        self.get_logger().info("Succeeded to set the baudrate.")

        for dxl_id in DXL_IDS:
            self.setupDynamixel(dxl_id)

    def setupDynamixel(self, dxl_id):
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_OPERATING_MODE, 3)
        self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
        self.get_logger().info(f"Motor {dxl_id} initialized.")

    def position_callback(self, msg):
        for i, dxl_id in enumerate(DXL_IDS):
            position = int(msg.position[i])
            dxl_comm_result = self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, position
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f"Failed to set position for Motor {dxl_id}")
            else:
                self.get_logger().info(f"Set [ID: {dxl_id}] [Goal Position: {position}]")

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
        for dxl_id in DXL_IDS:
            dynamixel_controller.packetHandler.write1ByteTxRx(
                dynamixel_controller.portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
        dynamixel_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

