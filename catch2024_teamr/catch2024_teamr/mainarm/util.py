import math

from rogidrive_msg.msg._rogidrive_message import RogidriveMessage
from rogilink3_interfaces.msg import Status
from rogidrive_msg.msg import RogidriveMultiArray
from catch2024_teamr_msgs.msg import MainArm

R_METER_TO_ROTATE_RATIO = 1


def theta_abs_to_count(theta: float, offset: float) -> int:
    return (float(theta) /
            2048 + offset) * 30


def theta_rad_to_rotate(theta: float) -> float:
    return theta / (2 * math.pi) * 30


def r_meter_to_rotate(r: float) -> float:
    return r * R_METER_TO_ROTATE_RATIO


def handtheta_to_pulsewidth(handtheta: float) -> float:
    return (handtheta + 1.57) / math.pi * 2000 + 500


def create_mainarm_status_msg(rogilink: Status,
                              rogidrive: RogidriveMultiArray) -> MainArm:
    msg = MainArm()

    theta: RogidriveMessage
    r: RogidriveMessage
    for i in rogidrive.data:
        if i.name == 'THETA':
            theta = i
        elif i.name == 'R':
            r = i

    msg.theta = theta.pos / 30 * 2 * math.pi
    msg.r = r.pos
    msg.lift = rogilink.motor[0].pos / R_METER_TO_ROTATE_RATIO

    return msg
