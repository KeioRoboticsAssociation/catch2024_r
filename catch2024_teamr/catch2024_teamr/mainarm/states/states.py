from enum import IntEnum

class States(IntEnum):
    INIT = 1
    GO_TARGET = 2
    CATCH = 3
    GO_SHOOT = 4
    SHOOT = 5
    END = 6