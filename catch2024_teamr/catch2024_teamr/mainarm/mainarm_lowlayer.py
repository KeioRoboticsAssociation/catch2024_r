from git import Union
import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm
import time
from std_msgs.msg import String
from rogilink3_interfaces.msg import Command, Status
from rogidrive_msg.msg import RogidriveMessage, RogidriveMultiArray

# 昇降DC, r ブラシレス, θ ブラシレス, ハンド サーボ1, サーボ2, サーボ3，開ループDC

ELEV_LIMIT_LOWER = 15
OMRON_SENSOR = 14


class MinarmLowLayer(Node):

    def __init__(self):
        super().__init__('mainarm_lowlayer')

        # メンバ変数の初期化
        self.rogilink_cmd = Command()
        self.rogilink_status: Union[Status, None] = None
        self.initialized = False

        # pub, sub の初期化
        self.subscription = self.create_subscription(
            MainArm, '/mainarm_pose', self.mainarm_lowlayer_callback, 10)
        self.rogilink_pub = self.create_publisher(Command,
                                                  '/rogilink3/command', 10)
        self.rogilink_sub = self.create_subscription(Status,
                                                     '/rogilink3/status',
                                                     self.rogilink_callback,
                                                     10)
        self.rogidrive_pub = self.create_publisher(RogidriveMessage,
                                                   '/odrive_msg', 10)
        self.rogidrive_sub = self.create_subscription(RogidriveMultiArray,
                                                      '/odrive_msg',
                                                      self.rogidrive_callback,
                                                      10)
        self.get_logger().info('mainarm_lowlayer has been started')
        self.timer = self.create_timer(0.01, self.timer_callback)

    def rogidrive_send(self, name: str, mode: int, vel: float, pos: float):
        msg = RogidriveMessage()
        msg.name = name
        msg.mode = mode
        msg.vel = vel
        msg.pos = pos
        self.rogidrive_pub.publish(msg)

    def timer_callback(self):
        if self.rogilink_status is not None and not (
                self.rogilink_status.limit >> ELEV_LIMIT_LOWER):
            self.rogilink_pub.publish(self.rogilink_cmd)

    def rogilink_callback(self, msg):
        self.rogilink_status = msg

    def rogidrive_callback(self, msg):
        self.rogidrive_status = msg

    def mainarm_lowlayer_callback(self, msg):
        self.get_logger().info('%s' % msg.data)
        self.rogidrive_send('THETA', 1, 0, msg.theta)  # TODO
        self.rogidrive_send('R', 1, 0, msg.r)  # TODO
        self.rogilink_cmd.motor[0].input_pos = msg.lift  # type: ignore # TODO
        self.rogilink_cmd.servo[  # type: ignore
            0].pulse_width_us = msg.hand1  # type: ignore # TODO

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
