from robot import RobotCategory
from senseArea import Region, EuclideanDistance, ManhattanDistance, Point


class UAV(RobotCategory):

    def __init__(self, rc_id, category, sensors, v, physical_property, intra=0.9):
        move_mode = "airplane"
        self.intra_factor = intra
        super().__init__(rc_id, category, sensors, move_mode, v, physical_property)

    def interD(self, reg1: Region, reg2: Region) -> float:
        return EuclideanDistance(reg1.represent_loc, reg2.represent_loc)

    def intraD(self, reg: Region) -> float:
        return 2 * sum(reg.len) * self.intra_factor

    def getLocation(self, reg1: Region, reg2: Region, percentage, tr) -> Region:
        x = (1 - percentage) * reg1.center[0] + percentage * reg2.center[0]
        y = (1 - percentage) * reg1.center[1] + percentage * reg2.center[1]
        p = Point(x, y)
        for reg in tr:
            if reg.inRange(p):
                return reg


class UV(RobotCategory):

    def __init__(self, rc_id, category, sensors, v, physical_property, intra=1.1):
        move_mode = "Land"
        self.intra_factor = intra
        super().__init__(rc_id, category, sensors, move_mode, v, physical_property)

    def interD(self, reg1: Region, reg2: Region) -> float:
        return ManhattanDistance(reg1.represent_loc, reg2.represent_loc)

    def intraD(self, reg: Region) -> float:
        return 2 * sum(reg.len) * self.intra_factor

    def getLocation(self, reg1: Region, reg2: Region, percentage, tr) -> Region:
        l1 = reg1.randomLoc()
        l2 = reg2.randomLoc()
        length = ManhattanDistance(l1, l2)
        x_p = abs(l1[0] - l2[0]) / length
        if percentage < x_p:
            r_loc_x = (1 - percentage) * l1[0] + percentage * l2[0]
            r_loc_y = l1[1]
        else:
            percentage = percentage - x_p
            r_loc_x = l2[0]
            r_loc_y = (1 - percentage) * l1[1] + percentage * l2[1]
        p = Point(r_loc_x, r_loc_y)
        for reg in tr:
            if reg.inRange(p):
                return reg


class Worker(UV):

    def __init__(self, rc_id, category, sensors, v, physical_property, intra=1.2):
        super().__init__(rc_id, category, sensors, v, physical_property, intra)


if __name__ == '__main__':
    region1 = Region(1, Point(0, 0), Point(1, 1))
    region2 = Region(2, Point(1, 0), Point(2, 1))
    print(region1, region2)
    uav_c = UAV(1, "DaJiang", None, 10, {"wight": 10, "height": 100, "width": 10, "length": 10})
    print(uav_c)
    print(uav_c.intraD(region1))
    print(uav_c.interD(region1, region2))
    print(uav_c.getLocation(region1, region2, 0, [region1, region2]))
    print(uav_c.getLocation(region1, region2, 1, [region1, region2]))
    print(uav_c.getLocation(region1, region2, 0.49, [region1, region2]))
