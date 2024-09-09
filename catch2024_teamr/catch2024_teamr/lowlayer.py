from typing import Union
import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm, Seiton
import time
import math
from std_msgs.msg import Bool
from rogilink3_interfaces.msg import Command, Status, MotorCommand
from rogidrive_msg.msg import RogidriveMessage, RogidriveMultiArray
from rogidrive_msg.msg import RogidriveSetCount
from .util import handtheta_to_pulsewidth, r_meter_to_rotate, y_meter_to_rotate
from .util import theta_abs_to_count, theta_rad_to_rotate
from .util import create_mainarm_status_msg, conveyer_count_to_rotate
from .util import flip_bool_to_pulsewidth, create_seiton_status_msg
# 昇降DC, r ブラシレス, θ ブラシレス, ハンド サーボ1, サーボ2, サーボ3，開ループDC

LIMIT_ELEV_LOWER = 15
LIMIT_CONVEYER_SENSOR = 0
SERVO_HAND_THETA = 0
SERVO_FLIP = 4
MOTOR_ELEV = 0

ABS_OFFSET = 0.0
THETA_MAX_VEL = 15.0
R_MAX_VEL = 5.0
Y_MAX_VEL = 5.0
CONVEYER_MAX_VEL = 5.0


class MinarmLowLayer(Node):

    def __init__(self):
        super().__init__('mainarm_lowlayer')

        # メンバ変数の初期化
        self.rogilink_cmd = Command()
        self.rogilink_status: Union[Status, None] = None
        self.rogidrive_status: Union[RogidriveMultiArray, None] = None
        self.mainarm_status = MainArm()
        self.prev_mainarm_cmd: Union[MainArm, None] = None
        self.prev_seiton_cmd: Union[Seiton, None] = None
        self.initialized = False

        # pub, sub の初期化
        self.mainarm_sub = self.create_subscription(
            MainArm, '/mainarm_target_pose', self.mainarm_lowlayer_callback, 10)
        self.mainarm_pub = self.create_publisher(
            MainArm, '/mainarm_status', 10)
        self.seiton_sub = self.create_subscription(
            Seiton, "/seiton_pose", self.seiton_lowlayer_callback, 10)
        self.seiton_pub = self.create_publisher(Seiton, "/seiton_status", 10)
        self.conveyer_sensor_pub = self.create_publisher(
            Bool, '/conveyer_sensor', 10)
        self.rogilink_pub = self.create_publisher(Command,
                                                  '/rogilink3/command', 10)
        self.rogilink_sub = self.create_subscription(Status,
                                                     '/rogilink3/status',
                                                     self.rogilink_callback,
                                                     10)
        self.rogidrive_pub = self.create_publisher(RogidriveMessage,
                                                   '/odrive_cmd', 10)
        self.rogidrive_enable = self.create_publisher(
            Bool, '/odrive_enable', 10)
        self.rogidrive_set_count = self.create_publisher(
            RogidriveSetCount, '/odrive_set_count', 10)
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

    def initialize(self):
        if self.rogilink_status is None or self.rogidrive_status is None:
            self.get_logger().warn(
                'rogilink or rogidrive status has not been received')
            return

        # rogidriveがすべてidleか
        rogidrive_initialized = True
        for i in self.rogidrive_status.data:
            if i.mode != -1:
                rogidrive_initialized = False

        # 昇降のリミットが押されていて、rogidriveが初期化されているか
        if ((self.rogilink_status.limit >> LIMIT_ELEV_LOWER) & 1 == 0
                and rogidrive_initialized):
            self.get_logger().info('Initialization complete')
            self.initialized = True
            self.rogidrive_set_count.publish(
                RogidriveSetCount(name='THETA',
                                  count=theta_abs_to_count(
                                      self.rogilink_status.abs_enc,
                                      ABS_OFFSET)))
            time.sleep(0.1)
            self.rogidrive_enable.publish(Bool(data=True))
        else:
            # 昇降を下限まで下げる
            self.get_logger().info('Initializing...')
            self.rogilink_cmd.motor[  # type: ignore
                MOTOR_ELEV].input_mode = (MotorCommand.COMMAND_VOL)
            self.rogilink_cmd.motor[  # type: ignore
                MOTOR_ELEV].input_vol = -0.1
            self.rogilink_pub.publish(self.rogilink_cmd)

    def rogilink_callback(self, msg: Status):
        self.rogilink_status = msg

    def rogidrive_callback(self, msg: RogidriveMultiArray):
        self.rogidrive_status = msg

    def timer_callback(self):
        if self.rogilink_status is None or self.rogidrive_status is None:
            self.get_logger().warn(
                'rogilink or rogidrive status has not been received')
            return

        if not self.initialized:
            self.initialize()
            return

        self.rogilink_pub.publish(self.rogilink_cmd)
        self.mainarm_pub.publish(create_mainarm_status_msg(
            self.rogilink_status, self.rogidrive_status))
        self.seiton_pub.publish(create_seiton_status_msg(
            self.rogilink_status, self.rogidrive_status))
        self.conveyer_sensor_pub.publish(
            Bool(data=(
                self.rogilink_status.limit >> LIMIT_CONVEYER_SENSOR) & 1 == 1))

    def mainarm_lowlayer_callback(self, msg: MainArm):
        if not self.initialized:
            return
        if self.prev_mainarm_cmd is None:
            self.prev_mainarm_cmd = msg
        self.get_logger().info('%s' % msg)
        if abs(msg.theta - self.prev_mainarm_cmd.theta) > 1.5 * math.pi:
            self.get_logger().error('delta theta is too large')
            return
        self.rogidrive_send('THETA', 1, THETA_MAX_VEL, theta_rad_to_rotate(
            msg.theta))  # rad, 角度境界に注意
        # self.rogidrive_send('R', 1, R_MAX_VEL,
        #                     r_meter_to_rotate(msg.r))  # 0 ~ 1m
        self.rogilink_cmd.motor[0].input_mode = (  # type: ignore
            MotorCommand.COMMAND_POS)
        self.rogilink_cmd.motor[0].input_pos = msg.lift  # type: ignore
        self.rogilink_cmd.servo[  # type: ignore
            SERVO_HAND_THETA].pulse_width_us = (
                handtheta_to_pulsewidth(msg.handtheta))
        self.prev_mainarm_cmd = msg

    def seiton_lowlayer_callback(self, msg: Seiton):
        if not self.initialized:
            return
        self.rogidrive_send('Y', 1, Y_MAX_VEL, y_meter_to_rotate(msg.y))
        self.rogidrive_send('CONVAYER', 1, CONVEYER_MAX_VEL,
                            conveyer_count_to_rotate(msg.conveyer))
        self.rogilink_cmd.servo[SERVO_FLIP].pulse_width_us = (  # type: ignore
            flip_bool_to_pulsewidth(
                msg.flip))
        self.prev_seiton_cmd = msg

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
