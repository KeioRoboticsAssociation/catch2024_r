import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm

class FullManual(Node):
    def __init__(self):
        
        

    def __del__(self):
        self.get_logger().info('full_manual has been destroyed')


def main(args=None):
    rclpy.init(args=args)
    full_manual = FullManual()
    try:
        rclpy.spin(full_manual)
    except KeyboardInterrupt:
        full_manual.get_logger().info(
            'Keyboard Interrupt (Ctrl+C) detected. Shutting down...')
    finally:
        full_manual.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()