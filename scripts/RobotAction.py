from enum import Enum
class RobotAction(Enum):
    none = 0
    move = 1
    direct = 2
    linear = 3
    orient = 4
    done = 5