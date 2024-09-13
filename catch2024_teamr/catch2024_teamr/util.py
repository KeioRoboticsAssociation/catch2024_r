import math

from rogidrive_msg.msg._rogidrive_message import RogidriveMessage
from rogilink3_interfaces.msg import Status
from rogidrive_msg.msg import RogidriveMultiArray
from catch2024_teamr_msgs.msg import MainArm, Seiton

R_METER_TO_ROTATE_RATIO = 13.5
CONVEYER_COUNT_TO_ROTATE_RATIO = -1.12
Y_METER_TO_ROTATE_RATIO = 1 / (0.03501 * math.pi)
Y_OFFSET = 0.606 - 0.07


def theta_abs_to_count(theta: int, offset: float) -> int:
    if theta - offset > 3072:
        return int(float(theta - offset - 4096) /
                   4096 * 30 * 8192)
    else:
        return int(float(theta - offset) /
                   4096 * 30 * 8192)


def theta_rad_to_rotate(theta: float) -> float:
    return theta / (2 * math.pi) * 30 * -1


def r_meter_to_rotate(r: float) -> float:
    return max(r * R_METER_TO_ROTATE_RATIO, 0.25)


def handtheta_to_pulsewidth(handtheta: float) -> int:
    return int((handtheta + 1.57) / math.pi * 1720 + 700)


def hand_to_pulsewidth(hand: int) -> int:
    if hand == 1:
        return 2050
    else:
        return 1000


def phi_to_pulsewidth():
    pass


def y_meter_to_rotate(meter: float) -> float:
    return (meter + Y_OFFSET) * Y_METER_TO_ROTATE_RATIO


def conveyer_count_to_rotate(count: int) -> float:
    return count * CONVEYER_COUNT_TO_ROTATE_RATIO


def flip_bool_to_pulsewidth(flip: bool) -> int:
    if flip:
        return 1930
    else:
        return 1100


def lift_to_rotate(lift: float) -> float:
    return lift * 2.7


def create_mainarm_status_msg(rogilink: Status,
                              rogidrive: RogidriveMultiArray) -> MainArm:
    msg = MainArm()

    theta = RogidriveMessage()
    r = RogidriveMessage()
    for i in rogidrive.data:
        if i.name == 'THETA':
            theta = i
        elif i.name == 'R':
            r = i

    msg.theta = theta.pos / 30 * 2 * math.pi * -1
    msg.r = r.pos / R_METER_TO_ROTATE_RATIO
    msg.lift = rogilink.motor[0].pos / 2.7

    return msg


def create_seiton_status_msg(rogilink: Status,
                             rogidrive: RogidriveMultiArray) -> Seiton:
    msg = Seiton()
    y = RogidriveMessage()
    conveyer = RogidriveMessage()
    for i in rogidrive.data:
        if i.name == 'Y':
            y = i
        elif i.name == 'CONVEYER':
            conveyer = i

    msg.y = y.pos / Y_METER_TO_ROTATE_RATIO - Y_OFFSET
    msg.conveyer = conveyer.pos / CONVEYER_COUNT_TO_ROTATE_RATIO

    return msg
