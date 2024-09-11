import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import Seiton, SeitonGUI, BoxStatus, BoxStatusMultiArray
from std_msgs.msg import Int8, Int8MultiArray
from sensor_msgs.msg import Joy
import math
from enum import IntEnum
from ..config.mode import Mode


class ButtonID(IntEnum):
    UP_50 = 0
    UP_10 = 1
    UP_5 = 2
    DOWN_5 = 3
    DOWN_10 = 4
    DOWN_50 = 5
    YURAYURA = 6
    GURIGURI = 7
    PATAPATA = 8
    SERVO_UP = 9
    SERVO_DOWN = 10
    FORWARD = 11
    REVERSE = 12
    DEFAULT = 13
    STOP = 14


INDEX_Y_POS = [
    -0.05393,
    -0.05393,
    -0.22393,
    -0.22393,
    -0.39393,
    -0.39393
]


class FullManual(Node):
    def __init__(self):
        super().__init__('semi_auto')
        self.pose_pub = self.create_publisher(
            Seiton, '/seiton/target_pose', 10)
        self.gui_sub = self.create_subscription(
            SeitonGUI, '/seiton/gui', self.gui_callback, 10)
        self.box_pub = self.create_publisher(
            BoxStatusMultiArray, '/seiton/box_status', 10)
        self.recom_pub = self.create_publisher(
            Int8MultiArray, '/seiton/recommendation', 10)

        self.joy_sub = self.create_subscription(
            Joy, '/seiton/joy', self.joy_callback, 5)
        self.index_sub = self.create_subscription(
            Int8, '/seiton/index', self.index_callback, 10)

        # hoge = [BoxStatus(ebishio=0 + 3 * i, yuzushio=1 + 3 *
        #                   i, norishio=2 + 3 * i) for i in range(6)]
        # self.box_pub.publish(BoxStatusMultiArray(data=hoge))

        # hoge = Int8MultiArray(data=[2, 2, 2])
        # self.recom_pub.publish(hoge)

        self.get_logger().info('semi_auto has been started')
        self.seiton_msg = Seiton()
        self.tmr = self.create_timer(0.01, self.timer_callback)

        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0]*8
        self.joy_msg.buttons = [0]*18
        self.previous_joy_msg = self.joy_msg

        self.seiton_msg.y = 0.0
        self.seiton_msg.mode = 0
        self.seiton_msg.conveyer = 0.0
        self.seiton_msg.flip = False

        self.cartesian_xy = [0, 0]

        self.boxes = BoxStatusMultiArray()
        self.boxes.data = [BoxStatus() for _ in range(6)]

        self.recom_id = Int8MultiArray()
        self.recom_id.data = [0, 0, 0]

        self.index = 0

    def joy_callback(self, msg):
        self.joy_msg = msg
        if self.joy_msg.buttons[ButtonID.UP_50]:
            self.seiton_msg.y += 0.05

        if self.joy_msg.buttons[ButtonID.UP_10]:
            self.seiton_msg.y += 0.01

        if self.joy_msg.buttons[ButtonID.UP_5]:
            self.seiton_msg.y += 0.005

        if self.joy_msg.buttons[ButtonID.DOWN_50]:
            self.seiton_msg.y -= 0.005

        if self.joy_msg.buttons[ButtonID.DOWN_10]:
            self.seiton_msg.y -= 0.01

        if self.joy_msg.buttons[ButtonID.DOWN_5]:
            self.seiton_msg.y -= 0.05

        if self.joy_msg.buttons[ButtonID.YURAYURA]:
            self.seiton_msg.mode = Mode.YURAYURA

        if self.joy_msg.buttons[ButtonID.GURIGURI]:
            self.seiton_msg.mode = Mode.GURIGURI

        if self.joy_msg.buttons[ButtonID.PATAPATA]:
            self.seiton_msg.mode = Mode.PATAPATA

        if self.joy_msg.buttons[ButtonID.SERVO_UP]:
            self.seiton_msg.flip = True

        if self.joy_msg.buttons[ButtonID.SERVO_DOWN]:
            self.seiton_msg.flip = False

        if self.joy_msg.buttons[ButtonID.DEFAULT]:
            self.seiton_msg.mode = Mode.DEFAULT

        if self.joy_msg.buttons[ButtonID.FORWARD]:
            self.seiton_msg.conveyer = 1.0
        
        if self.joy_msg.buttons[ButtonID.REVERSE]:
            self.seiton_msg.conveyer = -1.0

        if self.joy_msg.buttons[ButtonID.STOP]:
            self.seiton_msg.conveyer = 0.0

        self.pose_pub.publish(self.seiton_msg)


    def gui_callback(self, msg):
        delta = 1 if msg.plusminus else -1
        if msg.color == 'ebishio':
            self.boxes.data[msg.box_id].ebishio += delta
            if self.boxes.data[msg.box_id].ebishio < 0:
                self.boxes.data[msg.box_id].ebishio = 0
        elif msg.color == 'yuzushio':
            self.boxes.data[msg.box_id].yuzushio += delta
            if self.boxes.data[msg.box_id].yuzushio < 0:
                self.boxes.data[msg.box_id].yuzushio = 0
        elif msg.color == 'norishio':
            self.boxes.data[msg.box_id].norishio += delta
            if self.boxes.data[msg.box_id].norishio < 0:
                self.boxes.data[msg.box_id].norishio = 0
        self.box_pub.publish(self.boxes)
        self.calc_recom()

    def calc_recom(self):
        for i in range(6):
            if self.boxes.data[i].ebishio < 3:
                self.recom_id.data[0] = i
                break
        for i in range(6):
            if self.boxes.data[i].yuzushio < 3:
                self.recom_id.data[1] = i
                break 
        for i in range(6):
            if self.boxes.data[i].norishio< 3:
                self.recom_id.data[2] = i
                break
        self.recom_pub.publish(self.recom_id)

    def index_callback(self, msg):
        self.get_logger().info('index: %d' % msg.data)
        self.index = msg.data
        self.seiton_msg.mode = Mode.DEFAULT
        self.seiton_msg.y = INDEX_Y_POS[self.index]
        self.seiton_msg.flip = bool(self.index % 2)

        self.pose_pub.publish(self.seiton_msg)

    def timer_callback(self):

        self.previous_joy_msg = self.joy_msg

    def __del__(self):
        self.get_logger().info('semi_auto has been destroyed')


def main(args=None):
    rclpy.init(args=args)
    semi_auto = FullManual()
    try:
        rclpy.spin(semi_auto)
    except KeyboardInterrupt:
        semi_auto.get_logger().info(
            'Keyboard Interrupt (Ctrl+C) detected. Shutting down...')
    finally:
        semi_auto.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
