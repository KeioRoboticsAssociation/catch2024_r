import rclpy
from rclpy.node import Node
from catch2024_teamr_msgs.msg import MainArm, Seiton
from sensor_msgs.msg import Joy
import math
import time
from ..config.joy import Buttons, Axes, CoordinateMode
from .states.states import States
from std_msgs.msg import Int8
import csv


class SemiAuto(Node):
    def __init__(self):
        super().__init__('semi_auto')
        self.pose_pub = self.create_publisher(
            MainArm, '/mainarm/target_pose', 10)
        self.pose_sub = self.create_subscription(
            MainArm, '/mainarm/status', self.pose_callback, 5)
        self.pose_status = MainArm()

        def load_index_csv(file_path):
            index_data = []
            with open(file_path, 'r') as file:
                csv_reader = csv.DictReader(file)
                for row in csv_reader:
                    row.pop(next(iter(row)))
                    index_data.append([float(value) for value in row.values()])
            return index_data

        self.index_data = load_index_csv(
            '/home/rogi/ros2_ws/src/catch2024_r/catch2024_teamr/catch2024_teamr/config/index.csv')

        self.get_logger().info('index_data: %s' % self.index_data)

        self.state_pub = self.create_publisher(Int8, '/mainarm/state', 10)
        self.index_sub = self.create_subscription(
            Int8, '/index', self.index_callback, 5)
        self.seiton_sub = self.create_subscription(
            Seiton, '/seiton/target_pose', self.seiton_callback, 5)

        self.seiton_msg = Seiton()

        self.state = 0
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 5)
        self.get_logger().info('semi_auto has been started')
        self.mainarm_msg = MainArm()
        self.tmr = self.create_timer(0.01, self.timer_callback)
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0]*8
        self.joy_msg.buttons = [0]*11
        self.previous_joy_msg = self.joy_msg
        self.coordinate_mode = CoordinateMode.POLAR

        self.index = 0

        self.index_pose = [0, 0, 1]
        self.cartesian_xy = [0, 0]

        while True:
            self.loop()
            self.sleep_ms(1)

    def joy_callback(self, msg):
        self.joy_msg = msg

    def sleep_ms(self, ms):
        current_time = time.time()
        while time.time() - current_time < ms/1000:
            rclpy.spin_once(self)

    def set_state(self, state):
        self.state = state
        self.state_pub.publish(Int8(data=state))

    def send_target_xyz(self, xyz):
        self.cartesian_xy = [xyz[0], xyz[1]]
        self.mainarm_msg.r = math.sqrt(xyz[0]**2 + xyz[1]**2)
        self.mainarm_msg.theta = (
            math.atan2(xyz[1], xyz[0])) if not (
                -math.pi/2 > math.atan2(xyz[1], xyz[0]) > -math.pi
        ) else (math.atan2(xyz[1], xyz[0]) + 2*math.pi)
        self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta
        self.lift = xyz[2]
        self.pose_pub.publish(self.mainarm_msg)

    def send_target_rtz(self, rtz):
        self.mainarm_msg.r = rtz[0]
        self.mainarm_msg.theta = rtz[1]
        self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta
        self.mainarm_msg.lift = rtz[2]
        self.pose_pub.publish(self.mainarm_msg)

    def rtz_to_xyz(self, rtz):
        return [rtz[0]*math.cos(rtz[1]), rtz[0]*math.sin(rtz[1]), rtz[2]]

    def index_callback(self, msg):
        self.index = msg.data
        self.get_logger().info('index: %d' % self.index)

    def seiton_callback(self, msg):
        self.seiton_msg = msg

    def pose_callback(self, msg):
        self.pose_status = msg

    def wait_for_button(self, button):
        while True:
            rclpy.spin_once(self)
            if (self.joy_msg.buttons[button]
                != self.previous_joy_msg.buttons[button]
                    and self.joy_msg.buttons[button]):
                return

    def catch(self):
        self.mainarm_msg.lift = 0.0
        self.sleep_ms(1000)
        self.mainarm_msg.hand = 1
        self.pose_pub.publish(self.mainarm_msg)

    def release(self):
        self.mainarm_msg.lift = 1.0
        self.mainarm_msg.roller = True
        self.sleep_ms(1000)
        self.mainarm_msg.hand = 0
        self.mainarm_msg.roller = False
        self.pose_pub.publish(self.mainarm_msg)

    def loop(self):
        self.set_state(States.INIT)
        if self.state == States.INIT:
            self.get_logger().info('INIT')
            self.send_target_xyz([0.5, 0, 1])
            self.wait_for_button(Buttons.START)
            self.set_state(States.GO_TARGET)

        if self.state == States.GO_TARGET:
            self.get_logger().info('GO_TARGET')

            # 目標値のrtz座標
            rtz = self.rtz_to_xyz(self.index_data[self.index])
            # rだけ先に更新
            self.send_target_rtz([rtz[0], self.pose_status.theta, 0])
            # rが目標値に近づくまで待つ
            while abs(self.pose_status.r - rtz[0]) > 0.2:
                rclpy.spin_once(self)

            # θも更新
            self.send_target_rtz([rtz[0], rtz[1], 0])

            # self.send_target_xyz(self.index_data[self.index])
            self.wait_for_button(Buttons.START)
            self.set_state(States.CATCH)

        if self.state == States.CATCH:
            self.get_logger().info('CATCH')
            self.catch()
            self.wait_for_button(Buttons.START)
            self.set_state(States.GO_SHOOT)

        if self.state == States.GO_SHOOT:
            self.get_logger().info('GO_SHOOT')

            self.send_target_xyz([0.253, self.seiton_msg.y, 1])
            self.wait_for_button(Buttons.START)
            self.set_state(States.SHOOT)

        if self.state == States.SHOOT:
            self.get_logger().info('SHOOT')
            self.release()
            self.wait_for_button(Buttons.START)
            self.set_state(States.GO_TARGET)

        if self.state == States.END:
            self.get_logger().info('END')
            self.wait_for_button(Buttons.START)
            self.set_state(States.INIT)

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
            # self.get_logger().info('x: %f, y: %f' %
            #                        (self.cartesian_xy[0], self.cartesian_xy[1]))

        elif self.coordinate_mode == CoordinateMode.CARTESIAN:
            self.cartesian_xy[0] -= self.joy_msg.axes[Axes.LX]/100
            self.cartesian_xy[1] += self.joy_msg.axes[Axes.LY]/100

            self.mainarm_msg.r = math.sqrt(
                self.cartesian_xy[0]**2 + self.cartesian_xy[1]**2)
            if self.mainarm_msg.r < 0.0:
                self.mainarm_msg.r = 0.0
            elif self.mainarm_msg.r > 1.0:
                self.mainarm_msg.r = 1.0

            self.mainarm_msg.theta = math.atan2(
                self.cartesian_xy[1], self.cartesian_xy[0]
            ) if not (-math.pi/2 > math.atan2(
                self.cartesian_xy[1], self.cartesian_xy[0]) > -math.pi
            ) else math.atan2(
                self.cartesian_xy[1], self.cartesian_xy[0]) + 2*math.pi
            if self.mainarm_msg.theta < -math.pi/2:
                self.mainarm_msg.theta = -math.pi/2
            elif self.mainarm_msg.theta > math.pi*3/2:
                self.mainarm_msg.theta = math.pi*3/2

            self.mainarm_msg.handtheta -= self.joy_msg.axes[Axes.RX]/100
            if self.mainarm_msg.handtheta < -math.pi/2:
                self.mainarm_msg.handtheta = -math.pi/2
            elif self.mainarm_msg.handtheta > math.pi/2:
                self.mainarm_msg.handtheta = math.pi/2

            # self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta

            # self.get_logger().info('x: %f, y: %f' %
            #                        (self.cartesian_xy[0], self.cartesian_xy[1]))

        self.mainarm_msg.lift -= (1 - (self.joy_msg.axes[Axes.RT]+1)/2)/100
        if self.mainarm_msg.lift < 0.0:
            self.mainarm_msg.lift = 0.0

        self.mainarm_msg.lift += (1 - (self.joy_msg.axes[Axes.LT]+1)/2)/100
        if self.mainarm_msg.lift > 1.0:
            self.mainarm_msg.lift = 1.0

        if self.joy_msg.buttons[Buttons.LSTICK] == 1:
            self.mainarm_msg.handtheta = 1.57-self.mainarm_msg.theta

        if self.joy_msg.buttons[Buttons.B]:
            self.mainarm_msg.hand = 1

        if self.joy_msg.buttons[Buttons.A]:
            self.mainarm_msg.hand = 0

        if self.joy_msg.buttons[Buttons.X]:
            self.mainarm_msg.roller = True

        if self.joy_msg.buttons[Buttons.Y]:
            self.mainarm_msg.roller = False

        if (self.joy_msg.buttons[Buttons.HOME] !=
                self.previous_joy_msg.buttons[Buttons.HOME]
                and self.joy_msg.buttons[Buttons.HOME]):
            self.coordinate_mode = not self.coordinate_mode

        self.pose_pub.publish(self.mainarm_msg)
        self.previous_joy_msg = self.joy_msg

    def __del__(self):
        self.get_logger().info('node has been destroyed')


def main(args=None):
    rclpy.init(args=args)
    node = SemiAuto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'Keyboard Interrupt (Ctrl+C) detected. Shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
