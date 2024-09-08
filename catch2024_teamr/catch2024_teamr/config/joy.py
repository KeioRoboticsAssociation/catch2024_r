from enum import IntEnum

class Buttons(IntEnum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LB = 4
    RB = 5
    SELECT = 6
    START = 7
    HOME = 8
    LSTICK = 9
    RSTICK = 10
    SHARE = 11


class Axes(IntEnum):
    LX = 0
    LY = 1
    LT = 2
    RX = 3
    RY = 4
    RT = 5
    LDPAD = 6
    RDPAD = 7

class CoordinateMode(IntEnum):
    POLAR = 0
    CARTESIAN = 1