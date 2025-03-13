import rclpy
from rclpy.node import Node
from dynamixel_sdk import *                    
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual

import time
from pynput import keyboard

# Control table address for X series (except XL-320)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

# Protocol version
PROTOCOL_VERSION = 2.0  # Default Protocol version of DYNAMIXEL X series.
# BAUDRATE = 57600  # Default Baudrate of DYNAMIXEL X series
BAUDRATE = 1000000
DEVICE_NAME = "/dev/ttyUSB1"  # [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

# COMM_SUCCESS                = 0                             # Communication Success result value
# COMM_TX_FAIL                = -1001                         # Communication Tx Failed

DXL_ID1 = 1
DXL_ID2 = 2

'''=======================================================================================================
For controlling dual motor
======================================================================================================='''

dxl_error = 0
class MotorController(Node):
    def __init__(self):
        super().__init__('dynamixel_controller')
        self.portHandler = PortHandler(DEVICE_NAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.init_dynamixel()
        self.speed_subscriber = self.create_subscription(SetVelocityDual, '/motor_dual_speed', self.speed_callback, 10)

    def init_dynamixel(self):
        # Open Serial Port
        dxl_comm_result = self.portHandler.openPort()
        if dxl_comm_result == False:
            rclpy.logging.get_logger("dynamixel_controller").error("Failed to open the port!")
            return -1
        else:
            rclpy.logging.get_logger("dynamixel_controller").info("Succeeded to open the port.")

        # Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
        dxl_comm_result = self.portHandler.setBaudRate(BAUDRATE)
        if dxl_comm_result == False:
            rclpy.logging.get_logger("dynamixel_controller").error("Failed to set the baudrate!")
            return -1
        else:
            rclpy.logging.get_logger("dynamixel_controller").info("Succeeded to set the baudrate.")

        # Enable torque for both motors and Use Velocity Control Mode
        self.setupDynamixel(DXL_ID1)
        self.setupDynamixel(DXL_ID2)

    def setupDynamixel(self, dxl_id):
        # Use Velocity Control Mode
        dxl_comm_result = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            dxl_id,
            ADDR_OPERATING_MODE,
            1  # Velocity Control Mode
        )

        if dxl_comm_result == COMM_SUCCESS:
            rclpy.logging.get_logger("dynamixel_controller").error("Failed to set Velocity Control Mode.")
        else:
            rclpy.logging.get_logger("dynamixel_controller").info("Succeeded to set Velocity Control Mode.")

        # Enable Torque of DYNAMIXEL
        Torque_dxl_comm_result = self.packetHandler.write1ByteTxRx(
            self.portHandler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            1
        )

        if Torque_dxl_comm_result == COMM_SUCCESS:
            rclpy.logging.get_logger("dynamixel_controller").error("Failed to enable torque.")
        else:
            rclpy.logging.get_logger("dynamixel_controller").info("Succeeded to enable torque.")

    def speed_callback(self, msg):
        speed1 = msg.motorspeed1
        speed2 = msg.motorspeed2

        dxl_comm_result1 = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID1, ADDR_GOAL_VELOCITY, int(speed1))
        dxl_comm_result2 = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID2, ADDR_GOAL_VELOCITY, int(speed2))

        if dxl_comm_result1 != COMM_SUCCESS:
            rclpy.logging.get_logger('dynamixel_controller').info(self.packetHandler.getTxRxResult(dxl_comm_result1))
        elif dxl_error != 0:
            rclpy.logging.get_logger('dynamixel_controller').info(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rclpy.logging.get_logger('dynamixel_controller').info("Set [ID: %d] [Goal Velocity: %d]" % (DXL_ID1, msg.motorspeed1))

        if dxl_comm_result2 != COMM_SUCCESS:
            rclpy.logging.get_logger('dynamixel_controller').info(self.packetHandler.getTxRxResult(dxl_comm_result2))
        elif dxl_error != 0:
            rclpy.logging.get_logger('dynamixel_controller').info(self.packetHandler.getRxPacketError(dxl_error))
        else:
            rclpy.logging.get_logger('dynamixel_controller').info("Set [ID: %d] [Goal Velocity: %d]" % (DXL_ID2, msg.motorspeed2))
    

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
        # Loop to check if the 'esc' key is pressed
        while rclpy.ok():
            rclpy.spin_once(dynamixel_controller)
            time.sleep(0.1)  # Reduce CPU usage
            if not controller.listener.running:
                break  # Exit the loop if listener stopped
    finally:
        # Disable Torque of DYNAMIXEL
        dynamixel_controller.packetHandler.write1ByteTxRx(
            dynamixel_controller.portHandler,
            DXL_ID1,
            ADDR_TORQUE_ENABLE,
            0
        )
        dynamixel_controller.packetHandler.write1ByteTxRx(
            dynamixel_controller.portHandler,
            DXL_ID2,
            ADDR_TORQUE_ENABLE,
            0
        )
        # Clean up
        dynamixel_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
