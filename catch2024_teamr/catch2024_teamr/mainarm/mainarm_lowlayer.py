import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm
import time
from std_msgs.msg import String
from rogilink3_interfaces import *

class MinarmLowLayer(Node):
    def __init__(self):
        super().__init__('mainarm_lowlayer')
        self.subscription = self.create_subscription(MainArm,'main_arm',self.mainarm_lowlayer_callback,10)
        self.rogilink_pub = self.create_publisher(Command, '/rogilink3/command', 10)
        self.get_logger().info('mainarm_lowlayer has been started')

    def mainarm_lowlayer_callback(self, msg):
        self.get_logger().info('%s' % msg.data)

    def __del__(self):
        self.get_logger().info('mainarm_lowlayer has been destroyed')


def main(args=None):
    rclpy.init(args=args)
    mainarm_lowlayer = MinarmLowLayer()
    try:
        rclpy.spin(mainarm_lowlayer)
    except KeyboardInterrupt:
        mainarm_lowlayer.get_logger().info(
            'Keyboard Interrupt (Ctrl+C) detected. Shutting down...')
    finally:
        mainarm_lowlayer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
