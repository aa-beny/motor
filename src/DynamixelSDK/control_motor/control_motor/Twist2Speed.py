import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from dynamixel_sdk_custom_interfaces.msg import SetVelocityDual

class TwistToOKPublisherNode:
    def __init__(self):
        self.node = rclpy.create_node('twist_to_ok_publisher')

        # 创建一个订阅者，订阅Twist消息
        self.subscription = self.node.create_subscription(
            Twist,
            'cmd_vel',  # 这里应该替换成你实际接收Twist消息的话题名称
            self.twist_callback,
            10)
        self.subscription  # 避免unused variable警告

        # 创建一个发布者，发布String消息
        self.publisher = self.node.create_publisher(SetVelocityDual, '/motor_dual_speed', 10)

    def twist_callback(self, msg):
        # 当接收到Twist消息时调用此回调函数
        self.node.get_logger().info("Received Twist message: Linear = %f, Angular = %f" % (msg.linear.x, msg.angular.z))

        speed_wish_right = int((msg.angular.z*16)/2 + msg.linear.x)
        speed_wish_left = int(msg.linear.x*2-speed_wish_right)

        print(speed_wish_right, speed_wish_left)

        speed_msg = SetVelocityDual()

        speed_msg.motorspeed1 = -speed_wish_left
        speed_msg.motorspeed2 = -speed_wish_right

        self.publisher.publish(speed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToOKPublisherNode()
    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
