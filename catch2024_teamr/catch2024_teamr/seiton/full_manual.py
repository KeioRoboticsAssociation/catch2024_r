import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm
from sensor_msgs.msg import Joy
import math
from enum import IntEnum
from ..config.joy import *

class FullManual(Node):
    def __init__(self):
        super().__init__('full_manual')
        self.pose_pub = self.create_publisher(MainArm, '/seiton/target_pose', 10)
        self.joy_sub = self.create_subscription(
            Joy, '/seiton/joy', self.joy_callback, 5)
        self.get_logger().info('full_manual has been started')
        self.mainarm_msg = MainArm()
        self.tmr = self.create_timer(0.01, self.timer_callback)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0]*8
        self.joy_msg.buttons = [0]*11
        self.previous_joy_msg = self.joy_msg
        self.coordinate_mode = CoordinateMode.POLAR

        self.cartesian_xy = [0, 0]

    def joy_callback(self, msg):
        self.joy_msg = msg

    def timer_callback(self):
        if self.coordinate_mode == CoordinateMode.POLAR:
            self.mainarm_msg.theta += self.joy_msg.axes[Axes.LX]/100
            if self.mainarm_msg.theta < -math.pi/2:
                self.mainarm_msg.theta = -math.pi/2
            elif self.mainarm_msg.theta > math.pi*3/2:
                self.mainarm_msg.theta = math.pi*3/2

            self.mainarm_msg.r += self.joy_msg.axes[Axes.LY]/100
            if self.mainarm_msg.r < 0.0:
                self.mainarm_msg.r = 0.0
            elif self.mainarm_msg.r > 1.0:
                self.mainarm_msg.r = 1.0

            self.mainarm_msg.handtheta -= self.joy_msg.axes[Axes.RX]/100
            if self.mainarm_msg.handtheta < -math.pi/2:
                self.mainarm_msg.handtheta = -math.pi/2
            elif self.mainarm_msg.handtheta > math.pi/2:
                self.mainarm_msg.handtheta = math.pi/2

            self.cartesian_xy[0] = self.mainarm_msg.r * \
                math.cos(self.mainarm_msg.theta)
            self.cartesian_xy[1] = self.mainarm_msg.r * \
                math.sin(self.mainarm_msg.theta)
            self.get_logger().info('x: %f, y: %f' %
                                   (self.cartesian_xy[0], self.cartesian_xy[1]))

        elif self.coordinate_mode == CoordinateMode.CARTESIAN:
            self.cartesian_xy[0] -= self.joy_msg.axes[Axes.LX]/100
            self.cartesian_xy[1] += self.joy_msg.axes[Axes.LY]/100

            self.mainarm_msg.r = math.sqrt(
                self.cartesian_xy[0]**2 + self.cartesian_xy[1]**2)
            if self.mainarm_msg.r < 0.0:
                self.mainarm_msg.r = 0.0
            elif self.mainarm_msg.r > 1.0:
                self.mainarm_msg.r = 1.0

            self.mainarm_msg.theta = math.atan2(self.cartesian_xy[1], self.cartesian_xy[0]) if not -math.pi/2 > math.atan2(
                self.cartesian_xy[1], self.cartesian_xy[0]) > -math.pi else math.atan2(self.cartesian_xy[1], self.cartesian_xy[0]) + 2*math.pi
            if self.mainarm_msg.theta < -math.pi/2:
                self.mainarm_msg.theta = -math.pi/2
            elif self.mainarm_msg.theta > math.pi*3/2:
                self.mainarm_msg.theta = math.pi*3/2
            self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta

            self.get_logger().info('x: %f, y: %f' %
                                   (self.cartesian_xy[0], self.cartesian_xy[1]))

        self.mainarm_msg.lift -= (1 - (self.joy_msg.axes[Axes.RT]+1)/2)/100
        if self.mainarm_msg.lift < 0.0:
            self.mainarm_msg.lift = 0.0

        self.mainarm_msg.lift += (1 - (self.joy_msg.axes[Axes.LT]+1)/2)/100
        if self.mainarm_msg.lift > 1.0:
            self.mainarm_msg.lift = 1.0

        if self.joy_msg.buttons[Buttons.LSTICK] == 1:
            self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta

        if self.joy_msg.buttons[Buttons.B]:
            self.mainarm_msg.hand = True

        if self.joy_msg.buttons[Buttons.A]:
            self.mainarm_msg.hand = False

        if self.joy_msg.buttons[Buttons.X]:
            self.mainarm_msg.roller = True

        if self.joy_msg.buttons[Buttons.Y]:
            self.mainarm_msg.roller = False

        if self.joy_msg.buttons[Buttons.HOME] != self.previous_joy_msg.buttons[Buttons.HOME] and self.joy_msg.buttons[Buttons.HOME]:
            self.coordinate_mode = not self.coordinate_mode

        self.pose_pub.publish(self.mainarm_msg)
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
