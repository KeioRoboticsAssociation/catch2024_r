from typing import Union
import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm
import time
import math
from std_msgs.msg import String, Bool
from rogilink3_interfaces.msg import Command, Status, MotorCommand
from rogidrive_msg.msg import RogidriveMessage, RogidriveMultiArray
from rogidrive_msg.msg import RogidriveSetCount
from .util import handtheta_to_pulsewidth, r_meter_to_rotate
from .util import theta_abs_to_count, theta_rad_to_rotate
from .util import create_mainarm_status_msg
# 昇降DC, r ブラシレス, θ ブラシレス, ハンド サーボ1, サーボ2, サーボ3，開ループDC

LIMIT_ELEV_LOWER = 15
LIMIT_OMRON_SENSOR = 14
SERVO_HAND_THETA = 0
MOTOR_ELEV = 0

ABS_OFFSET = 0.0
THETA_MAX_VEL = 15
R_MAX_VEL = 5


class MinarmLowLayer(Node):

    def __init__(self):
        super().__init__('mainarm_lowlayer')

        # メンバ変数の初期化
        self.rogilink_cmd = Command()
        self.rogilink_status: Union[Status, None] = None
        self.rogidrive_status = Union[RogidriveMultiArray, None]
        self.mainarm_status = MainArm()
        self.prev_mainarm_cmd: Union[MainArm, None] = None
        self.initialized = False

        # pub, sub の初期化
        self.subscription = self.create_subscription(
            MainArm, '/mainarm_pose', self.mainarm_lowlayer_callback, 10)
        self.publisher = self.create_publisher(String, '/mainarm_status', 10)
        self.rogilink_pub = self.create_publisher(Command,
                                                  '/rogilink3/command', 10)
        self.rogilink_sub = self.create_subscription(Status,
                                                     '/rogilink3/status',
                                                     self.rogilink_callback,
                                                     10)
        self.rogidrive_pub = self.create_publisher(RogidriveMessage,
                                                   '/odrive_msg', 10)
        self.rogidrive_enable = self.create_publisher(
            Bool, '/odrive_enable', 10)
        self.rogidrive_set_count = self.create_publisher(
            RogidriveMessage, '/odrive_set_count', 10)
        self.rogidrive_sub = self.create_subscription(RogidriveMultiArray,
                                                      '/odrive_status',
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

    def rogilink_callback(self, msg: Status):
        self.rogilink_status = msg

    def rogidrive_callback(self, msg: RogidriveMultiArray):
        self.rogidrive_status = msg

    def timer_callback(self):
        if self.rogilink_status is None or self.rogidrive_status is None:
            self.get_logger().warn('rogilink or rogidrive status has not been received')
            return

        if self.initialized:
            self.rogilink_pub.publish(self.rogilink_cmd)
            return

        rogidrive_initialized = True
        for i in self.rogidrive_status.data:
            if i.mode != -1:
                rogidrive_initialized = False
        if ((self.rogilink_status.limit >> LIMIT_ELEV_LOWER) & 1 == 0
                and rogidrive_initialized):
            self.get_logger().info('Initialization completed')
            self.initialized = True
            self.rogidrive_set_count.publish(
                RogidriveSetCount(name='THETA',
                                  count=theta_abs_to_count(
                                      self.rogilink_status.abs_enc,
                                      ABS_OFFSET)))
            time.sleep(0.1)
            self.rogidrive_enable.publish(Bool(data=True))
        else:
            self.get_logger().info('Initializing...')
            self.rogilink_cmd.motor[MOTOR_ELEV].input_mode = (
                MotorCommand.COMMAND_VOL)
            self.rogilink_cmd.motor[MOTOR_ELEV].input_vol = -0.1
            self.rogilink_pub.publish(self.rogilink_cmd)

        self.publisher.publish(create_mainarm_status_msg(
            self.rogilink_status, self.rogidrive_status))

    def mainarm_lowlayer_callback(self, msg: MainArm):
        if not self.initialized:
            return
        if self.prev_mainarm_cmd is None:
            self.prev_mainarm_cmd = msg
        self.get_logger().info('%s' % msg.data)
        if msg.theta - self.prev_mainarm_cmd.theta > 1.5 * math.pi:
            self.get_logger().error('delta theta is too large')
            return
        self.rogidrive_send('THETA', 1, THETA_MAX_VEL, theta_rad_to_rotate(
            msg.theta))  # rad, 角度境界に注意
        self.rogidrive_send('R', 1, R_MAX_VEL,
                            r_meter_to_rotate(msg.r))  # 0 ~ 1m
        self.rogilink_cmd.motor[0].input_mode = MotorCommand.COMMAND_POS
        self.rogilink_cmd.motor[0].input_pos = msg.lift  # type: ignore
        self.rogilink_cmd.servo[  # type: ignore
            SERVO_HAND_THETA].pulse_width_us = (
                handtheta_to_pulsewidth(msg.handtheta))
        self.prev_mainarm_cmd = msg

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
