from robot import RobotCategory
from senseArea import Region, EuclideanDistance, ManhattanDistance, Point


class UAV(RobotCategory):

    def __init__(self, rc_id, category, sensors, v, physical_property, inter=0.9):
        move_mode = "airplane"
        self.inter_factor = inter
        super().__init__(rc_id, category, sensors, move_mode, v, physical_property)

    def interD(self, reg1: Region, reg2: Region) -> float:
        return EuclideanDistance(reg1.randomLoc(), reg2.randomLoc())

    def intraD(self, reg: Region) -> float:
        return 2*sum(reg.len)*self.inter_factor

    def getLocation(self, reg1: Region, reg2: Region, percentage, tr) -> Region:
        x = (1 - percentage) * reg1.center[0] + percentage * reg2.center[0]
        y = (1 - percentage) * reg1.center[1] + percentage * reg2.center[2]
        p = Point(x, y)
        for reg in tr:
            if reg.inRange(p):
                return reg


class UV(RobotCategory):

    def __init__(self, rc_id, category, sensors, v, physical_property, inter=1.1):
        move_mode = "Land"
        self.inter_factor = inter
        super().__init__(rc_id, category, sensors, move_mode, v, physical_property)

    def interD(self, reg1: Region, reg2: Region) -> float:
        return ManhattanDistance(reg1.randomLoc(), reg2.randomLoc())

    def intraD(self, reg: Region) -> float:
        return 2*sum(reg.len)*self.inter_factor

    def getLocation(self, reg1: Region, reg2: Region, percentage, tr) -> Region:
        l1 = reg1.randomLoc()
        l2 = reg2.randomLoc()
        length = ManhattanDistance(l1, l2)
        x_p = abs(l1[0] - l2[0]) / length
        if percentage < x_p:
            r_loc_x = (1-percentage)*l1[0] + percentage*l2[0]
            r_loc_y = l1[1]
        else:
            percentage = percentage - x_p
            r_loc_x = l2[0]
            r_loc_y = (1-percentage)*l1[1] + percentage*l2[1]
        p = Point(r_loc_x, r_loc_y)
        for reg in tr:
            if reg.inRange(p):
                return reg


class SmallUV(UV):

    def __init__(self, rc_id, category, sensors, v, physical_property, inter=1.2):
        super().__init__(rc_id, category, sensors, v, physical_property, inter)
