import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import Seiton
from sensor_msgs.msg import Joy
import math
from enum import IntEnum
from ..config.joy import *
from ..config.mode import Mode


class FullManual(Node):
    def __init__(self):
        super().__init__('full_manual')
        self.pose_pub = self.create_publisher(
            Seiton, '/seiton/target_pose', 10)
        self.joy_sub = self.create_subscription(
            Joy, '/seiton/joy', self.joy_callback, 5)
        self.get_logger().info('full_manual has been started')
        self.seiton_msg = Seiton()
        self.tmr = self.create_timer(0.01, self.timer_callback)

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0]*8
        self.joy_msg.buttons = [0]*12
        self.previous_joy_msg = self.joy_msg

        self.seiton_msg.y = 0.0
        self.seiton_msg.mode = 0
        self.seiton_msg.conveyer = 0
        self.seiton_msg.flip = False

        self.cartesian_xy = [0, 0]

    def joy_callback(self, msg):
        self.joy_msg = msg

    def timer_callback(self):
        if self.joy_msg.buttons[0]:
            self.seiton_msg.y += 0.05

        if self.joy_msg.buttons[1]:
            self.seiton_msg.y += 0.01

        if self.joy_msg.buttons[2]:
            self.seiton_msg.y += 0.005

        if self.joy_msg.buttons[3]:
            self.seiton_msg.y -= 0.005

        if self.joy_msg.buttons[4]:
            self.seiton_msg.y -= 0.01

        if self.joy_msg.buttons[5]:
            self.seiton_msg.y -= 0.05

        if self.joy_msg.buttons[6]:
            self.seiton_msg.mode = YURAYURA

        if self.joy_msg.buttons[7]:
            self.seiton_msg.mode = GURIGURI

        if self.joy_msg.buttons[8]:
            self.seiton_msg.mode = PATAPATA

        if self.joy_msg.buttons[9]:
            self.seiton_msg.flip = True

        if self.joy_msg.buttons[10]:
            self.seiton_msg.flip = False

        if self.joy_msg.buttons[11]:
            self.seiton_msg.conveyer += 1

        if self.joy_msg.buttons[12]:
            self.seiton_msg.conveyer -= 1

        self.pose_pub.publish(self.seiton_msg)
        self.previous_joy_msg = self.joy_msg

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
